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
/* verify and identify also match the first few frames while capture goes
 * on; a clear match there ends the press ~250 ms sooner. The bar is
 * stricter than E0C7F_MATCH_THRESHOLD since fewer frames are averaged;
 * on recorded samples it took 25 of 30 genuine presses and no wrong finger
 * came within 0.1 of it */
#define EARLY_FRAMES 3
#define EARLY_ACCEPT 0.62
/* Learning: a confident match adds the press to a few extra "learned"
 * templates for that finger, so matching adapts to how it is really
 * pressed. fprintd only stores prints at enrollment, so the driver keeps
 * these itself, next to fprintd's prints. Safeguards: the press must match
 * an *enrolled* template (never only a learned one) well above the
 * threshold, clearly beat every other finger, and be a good image; it
 * replaces the oldest of LEARN_SLOTS learned templates and never an
 * enrolled one. */
#define LEARN_DIR "/var/lib/fprint/.elan0c7f-learned"
#define LEARN_SLOTS 5
#define LEARN_MIN_SCORE 0.65  /* against an enrolled template */
#define LEARN_MAX_SCORE 0.90  /* already this close to a template: nothing new */
#define LEARN_MIN_MARGIN 0.20 /* over the best other finger */
#define LEARN_MIN_COVERAGE 0.75
#define LEARN_MIN_CLARITY 0.30
#define LEARN_MAGIC 0x314c3045 /* "E0L1" */
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
/* a finger already on the sensor when a capture starts is the last press
 * still lifting if a capture ended less than this ago (so a wrong finger held
 * down can't use up the tries at once); after longer it is a new press, e.g.
 * one made while the lock screen was restarting its check */
#define FRESH_PRESS_US (1500 * G_TIME_SPAN_MILLISECOND)
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

typedef struct _MatchJob MatchJob;

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

  gint64   last_capture_end; /* monotonic time */

  /* early match of the current press (see EARLY_FRAMES) */
  guint    capture_id;
  gboolean capturing;
  gboolean early_started;
  GTask   *early_task; /* set once the early match accepted the press */
};

G_DECLARE_FINAL_TYPE (FpiDeviceElan0c7f, fpi_device_elan0c7f, FPI, DEVICE_ELAN0C7F, FpDevice);
G_DEFINE_TYPE (FpiDeviceElan0c7f, fpi_device_elan0c7f, FP_TYPE_DEVICE);

static void start_capture (FpDevice *dev);
static void start_early_match (FpDevice *dev);

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
      /* untouched pixels read as zero, so a bright frame means a finger is
       * already there: the previous press still lifting, or (on an idle
       * sensor, not during enrollment) a new one to read at once */
      for (gsize i = 0; i + 1 < self->last_read_len; i += 2)
        sum += self->last_read[i] | (self->last_read[i + 1] << 8);
      if (sum / E0C7F_PIXELS > EMPTY_BG_MEAN &&
          (fpi_device_get_current_action (dev) == FPI_DEVICE_ACTION_ENROLL ||
           g_get_monotonic_time () - self->last_capture_end < FRESH_PRESS_US))
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
        if (self->nframes == EARLY_FRAMES && !self->early_started &&
            fpi_device_get_current_action (dev) != FPI_DEVICE_ACTION_ENROLL)
          {
            self->early_started = TRUE;
            start_early_match (dev);
          }
        if (self->early_task)
          fpi_ssm_jump_to_state (ssm, CAP_STOP);
        else if (self->nframes < CAPTURE_FRAMES)
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

struct _MatchJob
{
  FpiDeviceAction action;
  guint           capture_id;
  guint16        *frames;
  int             nframes;
  /* enrolled templates to compare against, with the print each belongs to */
  GArray         *templates;
  GPtrArray      *owners;
  GByteArray     *learned; /* per template: 1 if learned, 0 if enrolled */
  /* results */
  E0c7fTemplate   probe;
  gboolean        probe_ok;
  double          coverage, clarity;
  FpDeviceRetry   retry_reason; /* why probe_ok is FALSE */
  double         *scores; /* per template, same order as templates */
  double          best_score;
  FpPrint        *best_print;
};

static void
match_job_free (MatchJob *job)
{
  g_free (job->frames);
  g_free (job->scores);
  g_clear_pointer (&job->templates, g_array_unref);
  g_clear_pointer (&job->owners, g_ptr_array_unref);
  g_clear_pointer (&job->learned, g_byte_array_unref);
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

/* learned templates of print live in a file named after its data */
static gchar *
learned_path (FpPrint *print)
{
  g_autoptr(GVariant) data = NULL;
  g_autofree gchar *sum = NULL;
  const gchar *dir = g_getenv ("E0C7F_LEARN_DIR");

  g_object_get (print, "fpi-data", &data, NULL);
  if (!data)
    return NULL;
  sum = g_compute_checksum_for_data (G_CHECKSUM_SHA256, g_variant_get_data (data),
                                     g_variant_get_size (data));
  return g_strdup_printf ("%s/%s.bin", dir ? dir : LEARN_DIR, sum);
}

typedef struct
{
  guint32 magic, count, next;
} LearnedHeader;

/* the learned templates of print (none if there is no valid file) */
static GArray *
load_learned (FpPrint *print, guint *next)
{
  g_autofree gchar *path = learned_path (print);
  g_autofree gchar *contents = NULL;
  GArray *out = g_array_new (FALSE, FALSE, sizeof (E0c7fTemplate));
  LearnedHeader h;
  gsize len;

  *next = 0;
  if (!path || !g_file_get_contents (path, &contents, &len, NULL) || len < sizeof h)
    return out;
  memcpy (&h, contents, sizeof h);
  if (h.magic != LEARN_MAGIC || h.count > LEARN_SLOTS ||
      len != sizeof h + h.count * sizeof (E0c7fTemplate))
    return out;
  g_array_append_vals (out, contents + sizeof h, h.count);
  *next = h.next % LEARN_SLOTS;
  return out;
}

/* add probe to the learned templates of print, replacing the oldest */
static guint
store_learned (FpPrint *print, const E0c7fTemplate *probe)
{
  g_autofree gchar *path = learned_path (print);
  g_autoptr(GArray) learned = NULL;
  g_autoptr(GByteArray) out = g_byte_array_new ();
  g_autoptr(GError) error = NULL;
  g_autofree gchar *dir = NULL;
  LearnedHeader h;
  guint next, slot;

  if (!path)
    return G_MAXUINT;
  learned = load_learned (print, &next);
  if (learned->len < LEARN_SLOTS)
    {
      slot = learned->len;
      g_array_append_vals (learned, probe, 1);
    }
  else
    {
      slot = next;
      g_array_index (learned, E0c7fTemplate, slot) = *probe;
    }

  h = (LearnedHeader) { LEARN_MAGIC, learned->len, (slot + 1) % LEARN_SLOTS };
  g_byte_array_append (out, (const guint8 *) &h, sizeof h);
  g_byte_array_append (out, (const guint8 *) learned->data, learned->len * sizeof (E0c7fTemplate));

  dir = g_path_get_dirname (path);
  if (g_mkdir_with_parents (dir, 0700) < 0 ||
      !g_file_set_contents_full (path, (const gchar *) out->data, out->len,
                                 G_FILE_SET_CONTENTS_CONSISTENT, 0600, &error))
    {
      g_warning ("elan0c7f: could not save a learned template: %s",
                 error ? error->message : g_strerror (errno));
      return G_MAXUINT;
    }
  return slot;
}

/* append the templates stored in print, then its learned ones, to
 * templates/owners/learned */
static gboolean
add_print_templates (FpPrint *print, GArray *templates, GPtrArray *owners, GByteArray *learned)
{
  g_autoptr(GArray) extra = NULL;
  const guint8 yes = 1, no = 0;
  guint next;
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
          g_byte_array_append (learned, &no, 1);
        }
      g_variant_unref (img);
      g_variant_unref (mask);
    }

  extra = load_learned (print, &next);
  for (guint i = 0; i < extra->len; i++)
    {
      g_array_append_val (templates, g_array_index (extra, E0c7fTemplate, i));
      g_ptr_array_add (owners, print);
      g_byte_array_append (learned, &yes, 1);
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

/* after a match: learn the press if it is clearly this finger (see
 * LEARN_DIR for the safeguards) */
static void
maybe_learn (MatchJob *job)
{
  g_autoptr(GEnumClass) fingers = NULL;
  double enrolled = -1, same = -1, other = -1;
  GEnumValue *name;
  guint slot;

  if (job->action == FPI_DEVICE_ACTION_ENROLL || !job->probe_ok || !job->scores ||
      !job->best_print || job->best_score < E0C7F_MATCH_THRESHOLD)
    return;
  for (guint i = 0; i < job->templates->len; i++)
    {
      if (g_ptr_array_index (job->owners, i) != job->best_print)
        other = MAX (other, job->scores[i]);
      else if (job->learned->data[i])
        same = MAX (same, job->scores[i]);
      else
        enrolled = MAX (enrolled, job->scores[i]);
    }
  same = MAX (same, enrolled);
  if (enrolled < LEARN_MIN_SCORE || same > LEARN_MAX_SCORE ||
      enrolled - other < LEARN_MIN_MARGIN ||
      job->coverage < LEARN_MIN_COVERAGE || job->clarity < LEARN_MIN_CLARITY)
    return;

  slot = store_learned (job->best_print, &job->probe);
  if (slot == G_MAXUINT)
    return;
  fingers = g_type_class_ref (FP_TYPE_FINGER);
  name = g_enum_get_value (fingers, fp_print_get_finger (job->best_print));
  E0C7F_SCORE_LOG ("elan0c7f: learned this press for %s (slot %u of %u)",
                   name ? name->value_nick : "unknown", slot + 1, LEARN_SLOTS);
}

static void
report_match (FpDevice *dev, MatchJob *job)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);
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
      /* before reporting: the prints are freed once the action completes */
      maybe_learn (job);
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
      /* before reporting: the prints are freed once the action completes */
      maybe_learn (job);
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
match_done (GObject *source, GAsyncResult *res, gpointer user_data)
{
  report_match (FP_DEVICE (source), g_task_get_task_data (G_TASK (res)));
}

/* a match job for frames (taken over) against the prints of the current
 * action; NULL with error if the prints can't be read */
static MatchJob *
match_job_new (FpDevice *dev, guint16 *frames, int nframes, GError **error)
{
  FpiDeviceAction action = fpi_device_get_current_action (dev);
  MatchJob *job = g_new0 (MatchJob, 1);

  job->action = action;
  job->capture_id = FPI_DEVICE_ELAN0C7F (dev)->capture_id;
  job->frames = frames;
  job->nframes = nframes;
  job->templates = g_array_new (FALSE, FALSE, sizeof (E0c7fTemplate));
  job->owners = g_ptr_array_new ();
  job->learned = g_byte_array_new ();

  if (action == FPI_DEVICE_ACTION_VERIFY)
    {
      FpPrint *print;

      fpi_device_get_verify_data (dev, &print);
      if (!add_print_templates (print, job->templates, job->owners, job->learned))
        {
          match_job_free (job);
          g_set_error_literal (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                               "The stored print is not from this driver");
          return NULL;
        }
    }
  else if (action == FPI_DEVICE_ACTION_IDENTIFY)
    {
      GPtrArray *prints;

      fpi_device_get_identify_data (dev, &prints);
      for (guint i = 0; i < prints->len; i++)
        add_print_templates (g_ptr_array_index (prints, i), job->templates, job->owners,
                             job->learned);
    }
  return job;
}

static void
run_match (FpDevice *dev, MatchJob *job, GAsyncReadyCallback done)
{
  g_autoptr(GTask) task = g_task_new (dev, NULL, done, NULL);

  g_task_set_task_data (task, job, (GDestroyNotify) match_job_free);
  g_task_run_in_thread (task, match_thread);
}

static void
early_match_done (GObject *source, GAsyncResult *res, gpointer user_data)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (source);
  MatchJob *job = g_task_get_task_data (G_TASK (res));

  E0C7F_SCORE_LOG ("elan0c7f: early check: %d frames, probe %s, best %.3f (accept at %.3f)",
                   job->nframes, job->probe_ok ? "ok" : "rejected", job->best_score, EARLY_ACCEPT);
  /* keep it only if the press it belongs to is still being captured; the
   * capture then stops at its next frame and reports this result */
  if (self->capturing && job->capture_id == self->capture_id &&
      job->probe_ok && job->best_score >= EARLY_ACCEPT)
    self->early_task = g_object_ref (G_TASK (res));
}

static void
start_early_match (FpDevice *dev)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);
  MatchJob *job;

  job = match_job_new (dev, g_memdup2 (self->frames, EARLY_FRAMES * E0C7F_PIXELS * sizeof (guint16)),
                       EARLY_FRAMES, NULL);
  if (job)
    run_match (dev, job, early_match_done);
}

static void
capture_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);
  g_autoptr(GTask) early = g_steal_pointer (&self->early_task);
  MatchJob *job;

  self->capturing = FALSE;
  self->last_capture_end = g_get_monotonic_time ();
  fpi_device_report_finger_status_changes (dev, FP_FINGER_STATUS_NONE,
                                           FP_FINGER_STATUS_PRESENT | FP_FINGER_STATUS_NEEDED);
  if (error)
    {
      g_clear_pointer (&self->enroll_templates, g_array_unref);
      fpi_device_action_error (dev, error);
      return;
    }

  if (early)
    {
      report_match (dev, g_task_get_task_data (early));
      return;
    }

  job = match_job_new (dev, g_steal_pointer (&self->frames), self->nframes, &error);
  if (!job)
    {
      fpi_device_action_error (dev, error);
      return;
    }
  run_match (dev, job, match_done);
}

static void
start_capture (FpDevice *dev)
{
  FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);

  g_free (self->frames);
  self->frames = g_new0 (guint16, CAPTURE_FRAMES * E0C7F_PIXELS);
  self->nframes = 0;
  self->capture_id++;
  self->capturing = TRUE;
  self->early_started = FALSE;
  g_clear_object (&self->early_task);
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
