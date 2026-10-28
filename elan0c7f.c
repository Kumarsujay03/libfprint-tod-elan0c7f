/*
 * ELAN 04f3:0c7f press fingerprint sensor driver (libfprint TOD plugin)
 *
 * The sensor speaks the same USB protocol as the image sensors handled by the
 * upstream "elan" driver, but it is a small press sensor (52x150 pixels).
 * Its images are too small for minutiae matching, so this driver stores the
 * cleaned ridge images and matches them itself (see matcher.c).
 *
 * Copyright (C) 2026 Kumarsujay03
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#define FP_COMPONENT "elan0c7f"

#include <gmodule.h>

#include "drivers_api.h"
#include "matcher.h"

#define EP_CMD_OUT (0x1 | FPI_USB_ENDPOINT_OUT)
#define EP_CMD_IN (0x3 | FPI_USB_ENDPOINT_IN)
#define EP_IMG_IN (0x2 | FPI_USB_ENDPOINT_IN)

#define CMD_TIMEOUT 10000
/* no new frame within this time means the finger was lifted */
#define FINGER_OFF_TIMEOUT 200
#define FRAME_BYTES (E0C7F_PIXELS * 2)
/* finger frames averaged per press; more did not improve matching on
 * recorded samples and each one costs ~50 ms of contact */
#define CAPTURE_FRAMES 8
/* frames where the finger covers less than this are touch-down/lift */
#define MIN_FRAME_COVERAGE 0.3
/* presses below these are asked again. Coverage applies everywhere; the
 * clarity check only applies to enrollment, since some fingers are
 * naturally less clear (0.23-0.29) and must still get a quick no-match */
#define MIN_PRESS_COVERAGE 0.5
#define MIN_ENROLL_CLARITY 0.22
/* after this many retries of one spot the next press is accepted anyway */
#define MAX_SPOT_RETRIES 2
#define NR_ENROLL_STAGES 15
/* background mean above this means a finger is still on the sensor */
#define EMPTY_BG_MEAN 100
#define LIFT_POLL_MS 100
#define PRINT_VERSION 1

#ifdef E0C7F_LOG_SCORES
#define E0C7F_SCORE_LOG g_message
#else
#define E0C7F_SCORE_LOG fp_dbg
#endif

/* pre-scan replies */
#define FINGER_PRESENT 0x55

static const guint8 cmd_fw_ver[] = { 0x40, 0x19 };
static const guint8 cmd_sensor_dim[] = { 0x00, 0x0c };
static const guint8 cmd_get_image[] = { 0x00, 0x09 };
static const guint8 cmd_led_on[] = { 0x40, 0x31 };
static const guint8 cmd_pre_scan[] = { 0x40, 0x3f };
static const guint8 cmd_stop[] = { 0x00, 0x0b };

struct _FpiDeviceElan0c7f
{
  FpDevice parent;

  guint8  *last_read;
  gsize    last_read_len;

  guint16 *frames;
  int      nframes;

  /* enrolled templates collected so far */
  GArray  *enroll_templates;
  guint    spot_retries;
};

G_DECLARE_FINAL_TYPE (FpiDeviceElan0c7f, fpi_device_elan0c7f, FPI, DEVICE_ELAN0C7F, FpDevice);
G_DEFINE_TYPE (FpiDeviceElan0c7f, fpi_device_elan0c7f, FP_TYPE_DEVICE);

static void start_capture (FpDevice *dev);

/* ---- USB command helpers ---------------------------------------------- */

typedef struct
{
  guint8 endpoint;
  gsize  len;
  guint  timeout;
} CmdRead;

static void
cmd_read_cb (FpiUsbTransfer *transfer, FpDevice *dev,
             gpointer user_data, GError *error)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  g_free (self->last_read);
  self->last_read = g_memdup2 (transfer->buffer, transfer->actual_length);
  self->last_read_len = transfer->actual_length;
  fpi_ssm_next_state (transfer->ssm);
}

static void
cmd_write_cb (FpiUsbTransfer *transfer, FpDevice *dev,
              gpointer user_data, GError *error)
{
  g_autofree CmdRead *read = user_data;
  FpiUsbTransfer *in;

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  if (!read)
    {
      fpi_ssm_next_state (transfer->ssm);
      return;
    }

  in = fpi_usb_transfer_new (dev);
  in->ssm = transfer->ssm;
  in->short_is_error = TRUE;
  fpi_usb_transfer_fill_bulk (in, read->endpoint, read->len);
  fpi_usb_transfer_submit (in, read->timeout, fpi_device_get_cancellable (dev),
                           cmd_read_cb, NULL);
}

/* Send a 2-byte command; if resp_len > 0 read the reply from resp_ep.
 * The state machine advances once everything completed. */
static void
run_cmd (FpiSsm *ssm, FpDevice *dev, const guint8 *cmd,
         guint8 resp_ep, gsize resp_len, guint timeout)
{
  FpiUsbTransfer *out = fpi_usb_transfer_new (dev);
  CmdRead *read = NULL;

  if (resp_len)
    {
      read = g_new0 (CmdRead, 1);
      read->endpoint = resp_ep;
      read->len = resp_len;
      read->timeout = timeout;
    }

  out->ssm = ssm;
  out->short_is_error = TRUE;
  fpi_usb_transfer_fill_bulk (out, EP_CMD_OUT, 2);
  memcpy (out->buffer, cmd, 2);
  fpi_usb_transfer_submit (out, CMD_TIMEOUT, fpi_device_get_cancellable (dev),
                           cmd_write_cb, read);
}

/* ---- open / close ----------------------------------------------------- */

enum {
  OPEN_FW_VER,
  OPEN_SENSOR_DIM,
  OPEN_CHECK_DIM,
  OPEN_NUM_STATES,
};

static void
open_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case OPEN_FW_VER:
      run_cmd (ssm, dev, cmd_fw_ver, EP_CMD_IN, 2, CMD_TIMEOUT);
      break;

    case OPEN_SENSOR_DIM:
      fp_dbg ("firmware version 0x%02x%02x", self->last_read[0], self->last_read[1]);
      run_cmd (ssm, dev, cmd_sensor_dim, EP_CMD_IN, 4, CMD_TIMEOUT);
      break;

    case OPEN_CHECK_DIM:
      /* dimensions are reported zero-based: 51 x 149 */
      if (self->last_read[0] + 1 != E0C7F_W || self->last_read[2] + 1 != E0C7F_H)
        {
          fpi_ssm_mark_failed (ssm, fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                              "Unexpected sensor size %dx%d",
                                                              self->last_read[0] + 1,
                                                              self->last_read[2] + 1));
          return;
        }
      fpi_ssm_mark_completed (ssm);
      break;
    }
}

static void
open_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  fpi_device_open_complete (dev, error);
}

static void
dev_open (FpDevice *dev)
{
  GError *error = NULL;

  if (!g_usb_device_claim_interface (fpi_device_get_usb_device (dev), 0, 0, &error))
    {
      fpi_device_open_complete (dev, error);
      return;
    }

  fpi_ssm_start (fpi_ssm_new (dev, open_run_state, OPEN_NUM_STATES), open_done);
}

static void
dev_close (FpDevice *dev)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);
  GError *error = NULL;

  g_clear_pointer (&self->last_read, g_free);
  g_clear_pointer (&self->frames, g_free);
  g_clear_pointer (&self->enroll_templates, g_array_unref);
  g_usb_device_release_interface (fpi_device_get_usb_device (dev), 0, 0, &error);
  fpi_device_close_complete (dev, error);
}

/* ---- capture one press ------------------------------------------------ */

enum {
  CAP_BG_IMAGE,
  CAP_BG_CHECK,
  CAP_LED_ON,
  CAP_WAIT_FINGER,
  CAP_CHECK_FINGER,
  CAP_GET_IMAGE,
  CAP_SAVE_FRAME,
  CAP_STOP,
  CAP_NUM_STATES,
};

static void
wait_finger_cb (FpiUsbTransfer *transfer, FpDevice *dev,
                gpointer user_data, GError *error)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);

  /* once frames are coming in, a timeout means the finger was lifted */
  if (error && self->nframes > 0 &&
      g_error_matches (error, G_USB_DEVICE_ERROR, G_USB_DEVICE_ERROR_TIMED_OUT))
    {
      g_error_free (error);
      fpi_ssm_jump_to_state (transfer->ssm, CAP_STOP);
      return;
    }
  cmd_read_cb (transfer, dev, user_data, error);
}

static void
wait_finger_write_cb (FpiUsbTransfer *transfer, FpDevice *dev,
                      gpointer user_data, GError *error)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);
  FpiUsbTransfer *in;

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  in = fpi_usb_transfer_new (dev);
  in->ssm = transfer->ssm;
  in->short_is_error = TRUE;
  fpi_usb_transfer_fill_bulk (in, EP_CMD_IN, 1);
  /* wait for the first touch as long as it takes */
  fpi_usb_transfer_submit (in, self->nframes ? FINGER_OFF_TIMEOUT : 0,
                           fpi_device_get_cancellable (dev), wait_finger_cb, NULL);
}

static void
capture_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);
  FpiUsbTransfer *out;
  guint64 sum = 0;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case CAP_BG_IMAGE:
      run_cmd (ssm, dev, cmd_get_image, EP_IMG_IN, FRAME_BYTES, CMD_TIMEOUT);
      break;

    case CAP_BG_CHECK:
      /* untouched pixels read as zero, so a bright frame means the finger
       * from the previous press is still there */
      for (gsize i = 0; i + 1 < self->last_read_len; i += 2)
        sum += self->last_read[i] | (self->last_read[i + 1] << 8);
      if (sum / E0C7F_PIXELS > EMPTY_BG_MEAN)
        {
          fpi_device_report_finger_status_changes (dev, FP_FINGER_STATUS_PRESENT,
                                                   FP_FINGER_STATUS_NONE);
          fpi_ssm_jump_to_state_delayed (ssm, CAP_BG_IMAGE, LIFT_POLL_MS);
          return;
        }
      fpi_device_report_finger_status_changes (dev, FP_FINGER_STATUS_NEEDED,
                                               FP_FINGER_STATUS_PRESENT);
      fpi_ssm_next_state (ssm);
      break;

    case CAP_LED_ON:
      run_cmd (ssm, dev, cmd_led_on, 0, 0, 0);
      break;

    case CAP_WAIT_FINGER:
      out = fpi_usb_transfer_new (dev);
      out->ssm = ssm;
      out->short_is_error = TRUE;
      fpi_usb_transfer_fill_bulk (out, EP_CMD_OUT, 2);
      memcpy (out->buffer, cmd_pre_scan, 2);
      fpi_usb_transfer_submit (out, CMD_TIMEOUT, fpi_device_get_cancellable (dev),
                               wait_finger_write_cb, NULL);
      break;

    case CAP_CHECK_FINGER:
      if (self->last_read[0] != FINGER_PRESENT)
        {
          /* the sensor answers "not ready" instead of blocking; poll again */
          fpi_ssm_jump_to_state_delayed (ssm, CAP_WAIT_FINGER, 10);
          return;
        }
      if (self->nframes == 0)
        fpi_device_report_finger_status_changes (dev, FP_FINGER_STATUS_PRESENT,
                                                 FP_FINGER_STATUS_NONE);
      fpi_ssm_next_state (ssm);
      break;

    case CAP_GET_IMAGE:
      run_cmd (ssm, dev, cmd_get_image, EP_IMG_IN, FRAME_BYTES, CMD_TIMEOUT);
      break;

    case CAP_SAVE_FRAME:
      {
        guint16 *frame = self->frames + self->nframes * E0C7F_PIXELS;
        int covered = 0;

        for (int i = 0; i < E0C7F_PIXELS; i++)
          {
            frame[i] = self->last_read[2 * i] | (self->last_read[2 * i + 1] << 8);
            covered += frame[i] > 0;
          }
        /* only count frames where the finger is properly down */
        if (covered > MIN_FRAME_COVERAGE * E0C7F_PIXELS)
          self->nframes++;
        if (self->nframes < CAPTURE_FRAMES)
          fpi_ssm_jump_to_state (ssm, CAP_WAIT_FINGER);
        else
          fpi_ssm_next_state (ssm);
      }
      break;

    case CAP_STOP:
      run_cmd (ssm, dev, cmd_stop, 0, 0, 0);
      break;
    }
}

/* ---- matching (runs in a worker thread) ------------------------------- */

typedef struct
{
  FpiDeviceAction action;
  guint16        *frames;
  int             nframes;
  /* enrolled templates to compare against, with the print each belongs to */
  GArray         *templates;
  GPtrArray      *owners;
  /* results */
  E0c7fTemplate   probe;
  gboolean        probe_ok;
  double          coverage, clarity;
  FpDeviceRetry   retry_reason; /* why probe_ok is FALSE */
  double         *scores; /* per template, same order as templates */
  double          best_score;
  FpPrint        *best_print;
} MatchJob;

static void
match_job_free (MatchJob *job)
{
  g_free (job->frames);
  g_free (job->scores);
  g_clear_pointer (&job->templates, g_array_unref);
  g_clear_pointer (&job->owners, g_ptr_array_unref);
  g_free (job);
}

typedef struct
{
  MatchJob *job;
  guint     first, step;
  double    best;
  guint     best_idx;
} MatchSlice;

static gpointer
match_slice (gpointer data)
{
  MatchSlice *s = data;

  s->best = -1;
  for (guint i = s->first; i < s->job->templates->len; i += s->step)
    {
      double score = e0c7f_match (&g_array_index (s->job->templates, E0c7fTemplate, i),
                                  &s->job->probe);
      s->job->scores[i] = score;
      if (score > s->best)
        {
          s->best = score;
          s->best_idx = i;
        }
    }
  return NULL;
}

static void
match_thread (GTask *task, gpointer source, gpointer task_data,
              GCancellable *cancellable)
{
  MatchJob *job = task_data;
  guint nthreads;
  g_autofree MatchSlice *slices = NULL;
  g_autofree GThread **threads = NULL;

  job->probe_ok = e0c7f_template_from_frames (job->frames, job->nframes, &job->probe) == 0;
  job->retry_reason = FP_DEVICE_RETRY_CENTER_FINGER;
  if (job->probe_ok)
    {
      e0c7f_template_quality (&job->probe, &job->coverage, &job->clarity);
      /* a half-on press can't be judged; ask again instead of failing.
       * Enrollment applies its own, capped checks in match_done() */
      if (job->action != FPI_DEVICE_ACTION_ENROLL && job->coverage < MIN_PRESS_COVERAGE)
        job->probe_ok = FALSE;
    }
  job->best_score = -1;
  if (!job->probe_ok || !job->templates || job->templates->len == 0)
    {
      g_task_return_boolean (task, TRUE);
      return;
    }

  /* each comparison takes ~25 ms, so spread them over the CPUs */
  job->scores = g_new0 (double, job->templates->len);
  nthreads = MIN (g_get_num_processors (), job->templates->len);
  slices = g_new0 (MatchSlice, nthreads);
  threads = g_new0 (GThread *, nthreads);
  for (guint t = 0; t < nthreads; t++)
    {
      slices[t] = (MatchSlice) { .job = job, .first = t, .step = nthreads };
      threads[t] = g_thread_new ("elan0c7f-match", match_slice, &slices[t]);
    }
  for (guint t = 0; t < nthreads; t++)
    {
      g_thread_join (threads[t]);
      if (slices[t].best > job->best_score)
        {
          job->best_score = slices[t].best;
          job->best_print = g_ptr_array_index (job->owners, slices[t].best_idx);
        }
    }
  g_task_return_boolean (task, TRUE);
}

/* ---- print storage ---------------------------------------------------- */

static GVariant *
templates_to_variant (GArray *templates)
{
  GVariantBuilder builder;

  g_variant_builder_init (&builder, G_VARIANT_TYPE ("a(ayay)"));
  for (guint i = 0; i < templates->len; i++)
    {
      E0c7fTemplate *t = &g_array_index (templates, E0c7fTemplate, i);
      g_variant_builder_add (&builder, "(@ay@ay)",
                             g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, t->img,
                                                        E0C7F_PIXELS, 1),
                             g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, t->mask,
                                                        E0C7F_PIXELS, 1));
    }
  return g_variant_new ("(qqq@a(ayay))", PRINT_VERSION, E0C7F_W, E0C7F_H,
                        g_variant_builder_end (&builder));
}

/* append the templates stored in print to templates/owners */
static gboolean
add_print_templates (FpPrint *print, GArray *templates, GPtrArray *owners)
{
  g_autoptr(GVariant) data = NULL;
  g_autoptr(GVariant) list = NULL;
  guint16 version, w, h;
  GVariantIter iter;
  GVariant *img, *mask;

  g_object_get (print, "fpi-data", &data, NULL);
  if (!data || !g_variant_is_of_type (data, G_VARIANT_TYPE ("(qqqa(ayay))")))
    return FALSE;
  g_variant_get (data, "(qqq@a(ayay))", &version, &w, &h, &list);
  if (version != PRINT_VERSION || w != E0C7F_W || h != E0C7F_H)
    return FALSE;

  g_variant_iter_init (&iter, list);
  while (g_variant_iter_next (&iter, "(@ay@ay)", &img, &mask))
    {
      E0c7fTemplate t;
      gsize ilen, mlen;
      const guint8 *ip = g_variant_get_fixed_array (img, &ilen, 1);
      const guint8 *mp = g_variant_get_fixed_array (mask, &mlen, 1);

      if (ilen == E0C7F_PIXELS && mlen == E0C7F_PIXELS)
        {
          memcpy (t.img, ip, E0C7F_PIXELS);
          memcpy (t.mask, mp, E0C7F_PIXELS);
          g_array_append_val (templates, t);
          g_ptr_array_add (owners, print);
        }
      g_variant_unref (img);
      g_variant_unref (mask);
    }
  return TRUE;
}

/* ---- enroll / verify / identify --------------------------------------- */

/* "right-index-finger 0.62, left-index-finger 0.41" style summary */
static gchar *
describe_scores (MatchJob *job)
{
  g_autoptr(GString) out = g_string_new (NULL);
  g_autoptr(GEnumClass) fingers = g_type_class_ref (FP_TYPE_FINGER);

  if (!job->scores)
    return g_strdup ("none");

  for (guint i = 0; i < job->owners->len; i++)
    {
      FpPrint *print = g_ptr_array_index (job->owners, i);
      double best = -1;
      GEnumValue *name;

      /* report each print once, at its first template */
      if (i > 0 && g_ptr_array_index (job->owners, i - 1) == print)
        continue;
      for (guint j = i; j < job->owners->len && g_ptr_array_index (job->owners, j) == print; j++)
        best = MAX (best, job->scores[j]);

      name = g_enum_get_value (fingers, fp_print_get_finger (print));
      g_string_append_printf (out, "%s%s %.3f", out->len ? ", " : "",
                              name ? name->value_nick : "unknown", best);
    }
  return g_string_free (g_steal_pointer (&out), FALSE);
}

static void
match_done (GObject *source, GAsyncResult *res, gpointer user_data)
{
  FpDevice *dev = FP_DEVICE (source);
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);
  MatchJob *job = g_task_get_task_data (G_TASK (res));
  g_autofree gchar *scores = describe_scores (job);
  GError *retry = NULL;
  FpPrint *print;

  if (!job->probe_ok)
    retry = fpi_device_retry_new (job->retry_reason);
  /* builds with -DE0C7F_LOG_SCORES write every match score to the system log,
   * for measuring accuracy; normal builds keep them at debug level, since
   * exposing scores helps anyone trying to fool the matcher */
  E0C7F_SCORE_LOG ("elan0c7f: %s: %d frames, coverage %.2f, clarity %.2f, probe %s, "
             "%u templates, best %.3f (threshold %.3f): %s",
             job->action == FPI_DEVICE_ACTION_ENROLL ? "enroll" :
             job->action == FPI_DEVICE_ACTION_VERIFY ? "verify" : "identify",
             job->nframes, job->coverage, job->clarity, job->probe_ok ? "ok" : "rejected",
             job->templates ? job->templates->len : 0,
             job->best_score, E0C7F_MATCH_THRESHOLD, scores);

  switch (job->action)
    {
    case FPI_DEVICE_ACTION_ENROLL:
      /* ask for the same spot again if the press was half-on or unclear,
       * but only a couple of times so enrollment can't get stuck */
      if (!retry && self->spot_retries < MAX_SPOT_RETRIES)
        {
          if (job->coverage < MIN_PRESS_COVERAGE)
            retry = fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER);
          else if (job->clarity < MIN_ENROLL_CLARITY)
            retry = fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL);
        }
      if (retry)
        {
          self->spot_retries++;
          fpi_device_enroll_progress (dev, self->enroll_templates->len, NULL, retry);
          start_capture (dev);
          return;
        }
      self->spot_retries = 0;
      g_array_append_val (self->enroll_templates, job->probe);
      fpi_device_enroll_progress (dev, self->enroll_templates->len, NULL, NULL);
      if (self->enroll_templates->len < NR_ENROLL_STAGES)
        {
          start_capture (dev);
          return;
        }
      fpi_device_get_enroll_data (dev, &print);
      fpi_print_set_type (print, FPI_PRINT_RAW);
      g_object_set (print, "fpi-data", templates_to_variant (self->enroll_templates), NULL);
      g_clear_pointer (&self->enroll_templates, g_array_unref);
      fpi_device_enroll_complete (dev, g_object_ref (print), NULL);
      break;

    case FPI_DEVICE_ACTION_VERIFY:
      if (retry)
        fpi_device_verify_report (dev, FPI_MATCH_ERROR, NULL, retry);
      else
        fpi_device_verify_report (dev,
                                  job->best_score >= E0C7F_MATCH_THRESHOLD ?
                                  FPI_MATCH_SUCCESS : FPI_MATCH_FAIL,
                                  NULL, NULL);
      fpi_device_verify_complete (dev, NULL);
      break;

    case FPI_DEVICE_ACTION_IDENTIFY:
      if (retry)
        fpi_device_identify_report (dev, NULL, NULL, retry);
      else
        fpi_device_identify_report (dev,
                                    job->best_score >= E0C7F_MATCH_THRESHOLD ?
                                    job->best_print : NULL,
                                    NULL, NULL);
      fpi_device_identify_complete (dev, NULL);
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
capture_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);
  FpiDeviceAction action = fpi_device_get_current_action (dev);
  g_autoptr(GTask) task = NULL;
  MatchJob *job;

  fpi_device_report_finger_status_changes (dev, FP_FINGER_STATUS_NONE,
                                           FP_FINGER_STATUS_PRESENT | FP_FINGER_STATUS_NEEDED);
  if (error)
    {
      g_clear_pointer (&self->enroll_templates, g_array_unref);
      fpi_device_action_error (dev, error);
      return;
    }

  job = g_new0 (MatchJob, 1);
  job->action = action;
  job->frames = g_steal_pointer (&self->frames);
  job->nframes = self->nframes;
  job->templates = g_array_new (FALSE, FALSE, sizeof (E0c7fTemplate));
  job->owners = g_ptr_array_new ();

  if (action == FPI_DEVICE_ACTION_VERIFY)
    {
      FpPrint *print;

      fpi_device_get_verify_data (dev, &print);
      if (!add_print_templates (print, job->templates, job->owners))
        {
          match_job_free (job);
          fpi_device_action_error (dev, fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
          return;
        }
    }
  else if (action == FPI_DEVICE_ACTION_IDENTIFY)
    {
      GPtrArray *prints;

      fpi_device_get_identify_data (dev, &prints);
      for (guint i = 0; i < prints->len; i++)
        add_print_templates (g_ptr_array_index (prints, i), job->templates, job->owners);
    }

  task = g_task_new (dev, NULL, match_done, NULL);
  g_task_set_task_data (task, job, (GDestroyNotify) match_job_free);
  g_task_run_in_thread (task, match_thread);
}

static void
start_capture (FpDevice *dev)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);

  g_free (self->frames);
  self->frames = g_new0 (guint16, CAPTURE_FRAMES * E0C7F_PIXELS);
  self->nframes = 0;
  fpi_ssm_start (fpi_ssm_new (dev, capture_run_state, CAP_NUM_STATES), capture_done);
}

static void
dev_enroll (FpDevice *dev)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);

  g_clear_pointer (&self->enroll_templates, g_array_unref);
  self->enroll_templates = g_array_new (FALSE, FALSE, sizeof (E0c7fTemplate));
  self->spot_retries = 0;
  start_capture (dev);
}

static void
dev_verify_identify (FpDevice *dev)
{
  start_capture (dev);
}

/* ---- type registration ------------------------------------------------ */

static const FpIdEntry id_table[] = {
  { .vid = 0x04f3, .pid = 0x0c7f },
  { .vid = 0, .pid = 0 },
};

static void
fpi_device_elan0c7f_init (FpiDeviceElan0c7f *self)
{
}

static void
fpi_device_elan0c7f_class_init (FpiDeviceElan0c7fClass *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id = FP_COMPONENT;
  dev_class->full_name = "ELAN Fingerprint Sensor 04f3:0c7f";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->nr_enroll_stages = NR_ENROLL_STAGES;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  /* the sensor does not heat up during normal use */
  dev_class->temp_hot_seconds = -1;

  dev_class->open = dev_open;
  dev_class->close = dev_close;
  dev_class->enroll = dev_enroll;
  dev_class->verify = dev_verify_identify;
  dev_class->identify = dev_verify_identify;

  fpi_device_class_auto_initialize_features (dev_class);
}

/* entry point looked up by libfprint's TOD loader */
G_MODULE_EXPORT GType
fpi_tod_shared_driver_get_type (void)
{
  return fpi_device_elan0c7f_get_type ();
}
