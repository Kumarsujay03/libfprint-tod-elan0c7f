/*
 * ELAN 04f3:0c7f press fingerprint sensor driver (libfprint TOD plugin)
 *
 * The sensor speaks the same USB protocol as the image sensors handled by the
 * upstream "elan" driver, but it is a small press sensor (52x150 pixels).
 * Its images are too small for minutiae matching, so this driver stores the
 * cleaned ridge images and matches them itself (see matcher.c).
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
/* against an enrolled mosaic: right fingers there mostly score 0.57-0.75
 * and wrong ones stayed at or below 0.50, so a lower bar lets weaker
 * fingers learn too (the margin over other fingers still applies) */
#define LEARN_MIN_MOSAIC_SCORE 0.60
/* Mosaic prints also keep a library of their good presses (root only, next
 * to the learned mosaic), so presses that did not fit when they came can
 * be fitted in later: whenever learning grows the mosaic, the presses
 * still waiting (including enrolled ones that never stitched) are tried
 * again. The oldest press makes room when it is full. */
#define LIBRARY_SLOTS 100
#define LIBRARY_MAGIC 0x31503045 /* "E0P1" */
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
/* Enrollment keeps taking presses until the whole finger is known. Each
 * press is stitched into the finger's mosaic (see matcher.h); once
 * ENROLL_REPEATS presses in a row add almost no new area, the finger is
 * covered and enrollment ends. ENROLL_MAX_PRESSES is the most it asks for. */
#define ENROLL_MIN_PRESSES 10
#define ENROLL_MAX_PRESSES 30
#define ENROLL_REPEATS 4
#define ENROLL_REPEAT_AREA 0.10 /* of a press: a press adding less is a repeat */
#define ENROLL_MIN_AREA 3.0     /* presses' worth of finger before it may end */
/* nor while more presses than this wait to be stitched: a later press may
 * bridge to them and add a whole new part (one may be a slipped finger) */
#define ENROLL_MAX_ASIDE 1
/* a press that does not overlap the finger captured so far is asked for
 * again, closer in (so the mosaic stays one piece); after this many misses
 * in a row it is kept aside instead, so enrollment can't get stuck */
#define ENROLL_MAX_MISSES 3
#define NR_ENROLL_STAGES ENROLL_MAX_PRESSES
/* background mean above this means a finger is still on the sensor */
#define EMPTY_BG_MEAN 100
#define LIFT_POLL_MS 100
/* a finger already on the sensor when a capture starts is the last press
 * still lifting if a capture ended less than this ago (so a wrong finger held
 * down can't use up the tries at once); after longer it is a new press, e.g.
 * one made while the lock screen was restarting its check */
#define FRESH_PRESS_US (1500 * G_TIME_SPAN_MILLISECOND)
/* prints are saved as their presses (version 1), or since mosaics as the
 * presses, which of them are stitched, and the mosaic (version 2) */
#define PRINT_VERSION_PRESSES 1
#define PRINT_VERSION_MOSAIC 2
/* the mosaic as learning has updated it, next to the learned templates */
#define MOSAIC_MAGIC 0x314d3045 /* "E0M1" */
#define MOSAIC_MAX_SIDE 1024
/* builds with -DE0C7F_SAVE_SAMPLES keep the raw frames of every enrollment
 * press and a picture of the mosaic here (root only), so better image
 * processing can be tried later without enrolling again */
#define SAMPLES_DIR "/var/lib/fprint/.elan0c7f-samples"

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

  /* enrollment so far: the presses, which are stitched into the mosaic,
   * and how many presses in a row added nothing new */
  GArray      *enroll_templates;
  GByteArray  *enroll_stitched;
  E0c7fMosaic *enroll_mosaic;
  guint        enroll_repeats;
  guint        enroll_misses;
  guint        spot_retries;
  gchar       *samples_dir;

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

static void
enroll_state_clear (FpiDeviceElan0c7f *self)
{
  g_clear_pointer (&self->enroll_templates, g_array_unref);
  g_clear_pointer (&self->enroll_stitched, g_byte_array_unref);
  g_clear_pointer (&self->enroll_mosaic, e0c7f_mosaic_free);
  g_clear_pointer (&self->samples_dir, g_free);
}

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
  enroll_state_clear (self);
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
  /* the same for mosaics: per print the enrolled one, and the learned one
   * if learning has updated it */
  GPtrArray      *mosaics;
  GPtrArray      *mosaic_owners;
  GByteArray     *mosaic_learned;
  gint            next_ref; /* work queue over mosaics, then templates */
  guint           spot_retries;
  /* enrollment state, lent by the device while the thread stitches */
  GArray         *enroll_templates;
  GByteArray     *enroll_stitched;
  E0c7fMosaic    *enroll_mosaic;
  guint           enroll_misses;
  int             enroll_added;    /* pixels the press added */
  gboolean        enroll_fits;     /* the press is stitched on */
  gboolean        enroll_too_far;  /* not taken: it overlaps nothing known */
  double          enroll_fit;      /* how well it fits the mosaic */
  /* results */
  E0c7fTemplate   probe;
  gboolean        probe_ok;
  double          coverage, clarity;
  FpDeviceRetry   retry_reason; /* why probe_ok is FALSE */
  double         *scores; /* per template, same order as templates */
  double         *mosaic_scores;
  E0c7fPlace     *mosaic_places;
  double          best_score;
  FpPrint        *best_print;
};

static void
match_job_free (MatchJob *job)
{
  g_free (job->frames);
  g_free (job->scores);
  g_free (job->mosaic_scores);
  g_free (job->mosaic_places);
  g_clear_pointer (&job->templates, g_array_unref);
  g_clear_pointer (&job->owners, g_ptr_array_unref);
  g_clear_pointer (&job->learned, g_byte_array_unref);
  g_clear_pointer (&job->mosaics, g_ptr_array_unref);
  g_clear_pointer (&job->mosaic_owners, g_ptr_array_unref);
  g_clear_pointer (&job->mosaic_learned, g_byte_array_unref);
  g_clear_pointer (&job->enroll_templates, g_array_unref);
  g_clear_pointer (&job->enroll_stitched, g_byte_array_unref);
  g_clear_pointer (&job->enroll_mosaic, e0c7f_mosaic_free);
  g_free (job);
}

/* takes mosaics and templates off the job's queue until none are left */
static gpointer
match_worker (gpointer data)
{
  MatchJob *job = data;
  guint nm = job->mosaics->len, total = nm + job->templates->len, i;

  /* mosaics first: they take longest */
  while ((i = g_atomic_int_add (&job->next_ref, 1)) < total)
    if (i < nm)
      job->mosaic_scores[i] = e0c7f_mosaic_match (g_ptr_array_index (job->mosaics, i),
                                                  &job->probe, E0C7F_MATCH_OVERLAP,
                                                  &job->mosaic_places[i]);
    else
      job->scores[i - nm] = e0c7f_match (&g_array_index (job->templates, E0c7fTemplate, i - nm),
                                         &job->probe);
  return NULL;
}

/* why an enrollment press must be repeated, or -1 to take it */
static int
enroll_retry (MatchJob *job)
{
  if (!job->probe_ok)
    return job->retry_reason;
  /* ask for the same spot again if the press was half-on or unclear,
   * but only a couple of times so enrollment can't get stuck */
  if (job->spot_retries < MAX_SPOT_RETRIES)
    {
      if (job->coverage < MIN_PRESS_COVERAGE)
        return FP_DEVICE_RETRY_CENTER_FINGER;
      if (job->clarity < MIN_ENROLL_CLARITY)
        return FP_DEVICE_RETRY_GENERAL;
    }
  return -1;
}

/* add an enrollment press: stitch it onto the mosaic, then retry the
 * presses kept aside, since the mosaic may have grown towards them */
static void
stitch_press (MatchJob *job)
{
  const guint8 no = 0;
  guint8 *stitched;
  E0c7fPlace at;
  guint n;

  if (job->enroll_mosaic)
    {
      job->enroll_fit = e0c7f_mosaic_match (job->enroll_mosaic, &job->probe,
                                            E0C7F_STITCH_OVERLAP, &at);
      if (job->enroll_fit < E0C7F_STITCH_MIN && job->enroll_misses < ENROLL_MAX_MISSES)
        {
          job->enroll_too_far = TRUE;
          return;
        }
    }
  g_array_append_val (job->enroll_templates, job->probe);
  g_byte_array_append (job->enroll_stitched, &no, 1);
  n = job->enroll_templates->len;
  stitched = job->enroll_stitched->data;
  if (!job->enroll_mosaic)
    {
      job->enroll_mosaic = e0c7f_mosaic_new (&job->probe);
      job->enroll_added = e0c7f_mosaic_area (job->enroll_mosaic);
      stitched[n - 1] = 1;
    }
  for (gboolean grew = TRUE; grew;)
    {
      grew = FALSE;
      for (guint i = 0; i < n; i++)
        {
          const E0c7fTemplate *t = &g_array_index (job->enroll_templates, E0c7fTemplate, i);
          if (stitched[i] ||
              e0c7f_mosaic_match (job->enroll_mosaic, t, E0C7F_STITCH_OVERLAP, &at) < E0C7F_STITCH_MIN)
            continue;
          job->enroll_added += e0c7f_mosaic_add (job->enroll_mosaic, t, &at);
          stitched[i] = 1;
          grew = TRUE;
        }
    }
  job->enroll_fits = stitched[n - 1];
}

static void
match_thread (GTask *task, gpointer source, gpointer task_data,
              GCancellable *cancellable)
{
  MatchJob *job = task_data;
  guint nthreads, total;
  g_autofree GThread **threads = NULL;

  job->probe_ok = e0c7f_template_from_frames (job->frames, job->nframes, &job->probe) == 0;
  job->retry_reason = FP_DEVICE_RETRY_CENTER_FINGER;
  if (job->probe_ok)
    {
      e0c7f_template_quality (&job->probe, &job->coverage, &job->clarity);
      /* a half-on press can't be judged; ask again instead of failing.
       * Enrollment applies its own, capped checks in enroll_retry() */
      if (job->action != FPI_DEVICE_ACTION_ENROLL && job->coverage < MIN_PRESS_COVERAGE)
        job->probe_ok = FALSE;
    }
  job->best_score = -1;
  if (job->action == FPI_DEVICE_ACTION_ENROLL)
    {
      if (enroll_retry (job) < 0)
        stitch_press (job);
      g_task_return_boolean (task, TRUE);
      return;
    }
  total = job->templates->len + job->mosaics->len;
  if (!job->probe_ok || total == 0)
    {
      g_task_return_boolean (task, TRUE);
      return;
    }

  /* a template takes ~25 ms and a mosaic ~150 ms, so spread them over
   * the CPUs */
  job->scores = g_new0 (double, job->templates->len);
  job->mosaic_scores = g_new0 (double, job->mosaics->len);
  job->mosaic_places = g_new0 (E0c7fPlace, job->mosaics->len);
  nthreads = MIN (g_get_num_processors (), total);
  threads = g_new0 (GThread *, nthreads);
  for (guint t = 0; t < nthreads; t++)
    threads[t] = g_thread_new ("elan0c7f-match", match_worker, job);
  for (guint t = 0; t < nthreads; t++)
    g_thread_join (threads[t]);

  for (guint i = 0; i < job->templates->len; i++)
    if (job->scores[i] > job->best_score)
      {
        job->best_score = job->scores[i];
        job->best_print = g_ptr_array_index (job->owners, i);
      }
  for (guint i = 0; i < job->mosaics->len; i++)
    if (job->mosaic_scores[i] > job->best_score)
      {
        job->best_score = job->mosaic_scores[i];
        job->best_print = g_ptr_array_index (job->mosaic_owners, i);
      }
  g_task_return_boolean (task, TRUE);
}

/* ---- print storage ---------------------------------------------------- */

static GVariant *
template_list_variant (GArray *templates)
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
  return g_variant_builder_end (&builder);
}

/* a print stored as its presses only (before mosaics); the tests still
 * make these */
G_GNUC_UNUSED static GVariant *
templates_to_variant (GArray *templates)
{
  return g_variant_new ("(qqq@a(ayay))", PRINT_VERSION_PRESSES, E0C7F_W, E0C7F_H,
                        template_list_variant (templates));
}

static GVariant *
mosaic_to_variant (const E0c7fMosaic *m)
{
  return g_variant_new ("(qq@ay@ay)", m->w, m->h,
                        g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, m->img, m->w * m->h, 1),
                        g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, m->weight, m->w * m->h, 1));
}

/* a print stored as its presses, which of them are stitched, and the mosaic */
static GVariant *
mosaic_print_to_variant (GArray *templates, GByteArray *stitched, const E0c7fMosaic *m)
{
  return g_variant_new ("(qqq@a(ayay)@ay@(qqayay))", PRINT_VERSION_MOSAIC, E0C7F_W, E0C7F_H,
                        template_list_variant (templates),
                        g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, stitched->data,
                                                   stitched->len, 1),
                        mosaic_to_variant (m));
}

static E0c7fMosaic *
mosaic_from_variant (GVariant *v)
{
  g_autoptr(GVariant) img = NULL, weight = NULL;
  guint16 w, h;
  gsize ilen, wlen;
  const guint8 *ip, *wp;

  g_variant_get (v, "(qq@ay@ay)", &w, &h, &img, &weight);
  ip = g_variant_get_fixed_array (img, &ilen, 1);
  wp = g_variant_get_fixed_array (weight, &wlen, 1);
  if (!w || !h || w > MOSAIC_MAX_SIDE || h > MOSAIC_MAX_SIDE ||
      ilen != (gsize) w * h || wlen != (gsize) w * h)
    return NULL;
  return e0c7f_mosaic_wrap (w, h, g_memdup2 (ip, ilen), g_memdup2 (wp, wlen));
}

/* learned templates of print live in a file named after its data */
static gchar *
learned_file (FpPrint *print, const char *ext)
{
  g_autoptr(GVariant) data = NULL;
  g_autofree gchar *sum = NULL;
  const gchar *dir = g_getenv ("E0C7F_LEARN_DIR");

  g_object_get (print, "fpi-data", &data, NULL);
  if (!data)
    return NULL;
  sum = g_compute_checksum_for_data (G_CHECKSUM_SHA256, g_variant_get_data (data),
                                     g_variant_get_size (data));
  return g_strdup_printf ("%s/%s.%s", dir ? dir : LEARN_DIR, sum, ext);
}

static gchar *
learned_path (FpPrint *print)
{
  return learned_file (print, "bin");
}

typedef struct
{
  guint32 magic, w, h, learned;
} MosaicHeader;

/* a learned mosaic file, or NULL if there is none or it is damaged */
static E0c7fMosaic *
load_mosaic_file (const gchar *path, guint *learned)
{
  g_autofree gchar *contents = NULL;
  MosaicHeader h;
  gsize len, n;

  if (!path || !g_file_get_contents (path, &contents, &len, NULL) || len < sizeof h)
    return NULL;
  memcpy (&h, contents, sizeof h);
  n = (gsize) h.w * h.h;
  if (h.magic != MOSAIC_MAGIC || !h.w || !h.h || h.w > MOSAIC_MAX_SIDE ||
      h.h > MOSAIC_MAX_SIDE || len != sizeof h + 2 * n)
    return NULL;
  if (learned)
    *learned = h.learned;
  return e0c7f_mosaic_wrap (h.w, h.h, g_memdup2 (contents + sizeof h, n),
                            g_memdup2 (contents + sizeof h + n, n));
}

/* write a private file atomically, creating its directory */
static gboolean
store_private (const gchar *path, GByteArray *data, const char *what)
{
  g_autofree gchar *dir = g_path_get_dirname (path);
  g_autoptr(GError) error = NULL;

  if (g_mkdir_with_parents (dir, 0700) < 0 ||
      !g_file_set_contents_full (path, (const gchar *) data->data, data->len,
                                 G_FILE_SET_CONTENTS_CONSISTENT, 0600, &error))
    {
      g_warning ("elan0c7f: could not save the %s: %s", what,
                 error ? error->message : g_strerror (errno));
      return FALSE;
    }
  return TRUE;
}

static gboolean
store_mosaic_file (const gchar *path, const E0c7fMosaic *m, guint learned)
{
  g_autoptr(GByteArray) out = g_byte_array_new ();
  MosaicHeader h = { MOSAIC_MAGIC, m->w, m->h, learned };

  g_byte_array_append (out, (const guint8 *) &h, sizeof h);
  g_byte_array_append (out, m->img, m->w * m->h);
  g_byte_array_append (out, m->weight, m->w * m->h);
  return store_private (path, out, "learned mosaic");
}

/* the mosaic of print as learning has updated it, or NULL */
static E0c7fMosaic *
load_learned_mosaic (FpPrint *print, guint *learned)
{
  g_autofree gchar *path = learned_file (print, "mosaic");

  return path ? load_mosaic_file (path, learned) : NULL;
}

/* ---- library of good presses ------------------------------------------ */

typedef struct
{
  guint32 magic, count, next;
} LibraryHeader;

typedef struct
{
  guint8        placed; /* already blended into the learned mosaic */
  guint8        pad[3];
  E0c7fTemplate t;
} LibraryPress;

static GArray *
load_library (const gchar *path, guint *next)
{
  g_autofree gchar *contents = NULL;
  GArray *out = g_array_new (FALSE, FALSE, sizeof (LibraryPress));
  LibraryHeader h;
  gsize len;

  *next = 0;
  if (!g_file_get_contents (path, &contents, &len, NULL) || len < sizeof h)
    return out;
  memcpy (&h, contents, sizeof h);
  if (h.magic != LIBRARY_MAGIC || h.count > LIBRARY_SLOTS ||
      len != sizeof h + h.count * sizeof (LibraryPress))
    return out;
  g_array_append_vals (out, contents + sizeof h, h.count);
  *next = h.next % LIBRARY_SLOTS;
  return out;
}

static gboolean
store_library (const gchar *path, GArray *lib, guint next)
{
  g_autoptr(GByteArray) out = g_byte_array_new ();
  LibraryHeader h = { LIBRARY_MAGIC, lib->len, next };

  g_byte_array_append (out, (const guint8 *) &h, sizeof h);
  g_byte_array_append (out, (const guint8 *) lib->data, lib->len * sizeof (LibraryPress));
  return store_private (path, out, "press library");
}

/* add a press, replacing the oldest once the library is full */
static void
library_add (GArray *lib, guint *next, const E0c7fTemplate *t, gboolean placed)
{
  LibraryPress p = { .placed = placed, .t = *t };

  if (lib->len < LIBRARY_SLOTS)
    g_array_append_val (lib, p);
  else
    {
      g_array_index (lib, LibraryPress, *next) = p;
      *next = (*next + 1) % LIBRARY_SLOTS;
    }
}

/* fit in the presses that are still waiting, as enrollment does: retry
 * until no more fit. Returns how many were fitted. */
static guint
refit_waiting (E0c7fMosaic *m, GArray *lib)
{
  guint fitted = 0;
  E0c7fPlace at;

  for (gboolean grew = TRUE; grew;)
    {
      grew = FALSE;
      for (guint i = 0; i < lib->len; i++)
        {
          LibraryPress *p = &g_array_index (lib, LibraryPress, i);
          if (p->placed ||
              e0c7f_mosaic_match (m, &p->t, E0C7F_STITCH_OVERLAP, &at) < E0C7F_STITCH_MIN)
            continue;
          e0c7f_mosaic_add (m, &p->t, &at);
          p->placed = 1;
          fitted++;
          grew = TRUE;
        }
    }
  return fitted;
}

static guint
library_waiting (GArray *lib)
{
  guint n = 0;

  for (guint i = 0; i < lib->len; i++)
    n += !g_array_index (lib, LibraryPress, i).placed;
  return n;
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

static void
add_template (MatchJob *job, FpPrint *print, const E0c7fTemplate *t, guint8 learned)
{
  g_array_append_vals (job->templates, t, 1);
  g_ptr_array_add (job->owners, print);
  g_byte_array_append (job->learned, &learned, 1);
}

static void
add_mosaic (MatchJob *job, FpPrint *print, E0c7fMosaic *m, guint8 learned)
{
  g_ptr_array_add (job->mosaics, m);
  g_ptr_array_add (job->mosaic_owners, print);
  g_byte_array_append (job->mosaic_learned, &learned, 1);
}

/* add what print holds to the job: for a mosaic print its mosaic, the
 * learned one if there is one, and the presses that could not be
 * stitched; for an older print its presses and learned templates */
static gboolean
add_print_templates (FpPrint *print, MatchJob *job)
{
  g_autoptr(GVariant) data = NULL;
  g_autoptr(GVariant) list = NULL;
  g_autoptr(GVariant) stitched_v = NULL;
  g_autoptr(GVariant) mosaic_v = NULL;
  const guint8 *stitched = NULL;
  gsize nstitched = 0;
  E0c7fMosaic *mosaic = NULL;
  guint16 version, w, h;
  GVariantIter iter;
  GVariant *img, *mask;
  guint i = 0;

  g_object_get (print, "fpi-data", &data, NULL);
  if (data && g_variant_is_of_type (data, G_VARIANT_TYPE ("(qqqa(ayay))")))
    g_variant_get (data, "(qqq@a(ayay))", &version, &w, &h, &list);
  else if (data && g_variant_is_of_type (data, G_VARIANT_TYPE ("(qqqa(ayay)ay(qqayay))")))
    {
      g_variant_get (data, "(qqq@a(ayay)@ay@(qqayay))", &version, &w, &h, &list,
                     &stitched_v, &mosaic_v);
      stitched = g_variant_get_fixed_array (stitched_v, &nstitched, 1);
      if (version != PRINT_VERSION_MOSAIC ||
          nstitched != g_variant_n_children (list) ||
          !(mosaic = mosaic_from_variant (mosaic_v)))
        return FALSE;
    }
  else
    return FALSE;
  if ((!mosaic && version != PRINT_VERSION_PRESSES) || w != E0C7F_W || h != E0C7F_H)
    {
      e0c7f_mosaic_free (mosaic);
      return FALSE;
    }

  g_variant_iter_init (&iter, list);
  for (; g_variant_iter_next (&iter, "(@ay@ay)", &img, &mask); i++)
    {
      E0c7fTemplate t;
      gsize ilen, mlen;
      const guint8 *ip = g_variant_get_fixed_array (img, &ilen, 1);
      const guint8 *mp = g_variant_get_fixed_array (mask, &mlen, 1);

      /* stitched presses are already in the mosaic */
      if (ilen == E0C7F_PIXELS && mlen == E0C7F_PIXELS && !(stitched && stitched[i]))
        {
          memcpy (t.img, ip, E0C7F_PIXELS);
          memcpy (t.mask, mp, E0C7F_PIXELS);
          add_template (job, print, &t, 0);
        }
      g_variant_unref (img);
      g_variant_unref (mask);
    }

  if (mosaic)
    {
      E0c7fMosaic *learned = load_learned_mosaic (print, NULL);

      add_mosaic (job, print, mosaic, 0);
      if (learned)
        add_mosaic (job, print, learned, 1);
    }
  else
    {
      g_autoptr(GArray) extra = NULL;
      guint next;

      extra = load_learned (print, &next);
      for (guint k = 0; k < extra->len; k++)
        add_template (job, print, &g_array_index (extra, E0c7fTemplate, k), 1);
    }
  return TRUE;
}

/* ---- enroll / verify / identify --------------------------------------- */

/* best score per print of the job: templates and mosaics */
static double
print_best (MatchJob *job, FpPrint *print)
{
  double best = -1;

  for (guint i = 0; i < job->templates->len; i++)
    if (g_ptr_array_index (job->owners, i) == print)
      best = MAX (best, job->scores[i]);
  for (guint i = 0; i < job->mosaics->len; i++)
    if (g_ptr_array_index (job->mosaic_owners, i) == print)
      best = MAX (best, job->mosaic_scores[i]);
  return best;
}

/* the bar the best print must reach: stricter for a mosaic print */
static double
match_threshold (MatchJob *job)
{
  if (job->best_print && g_ptr_array_find (job->mosaic_owners, job->best_print, NULL))
    return E0C7F_MOSAIC_THRESHOLD;
  return E0C7F_MATCH_THRESHOLD;
}

static gboolean
is_match (MatchJob *job)
{
  return job->best_print && job->best_score >= match_threshold (job);
}

/* "right-index-finger 0.62, left-index-finger 0.41" style summary */
static gchar *
describe_scores (MatchJob *job)
{
  g_autoptr(GString) out = g_string_new (NULL);
  g_autoptr(GEnumClass) fingers = g_type_class_ref (FP_TYPE_FINGER);
  g_autoptr(GPtrArray) prints = g_ptr_array_new ();

  if (!job->scores)
    return g_strdup ("none");

  for (guint i = 0; i < job->mosaic_owners->len; i++)
    if (!g_ptr_array_find (prints, g_ptr_array_index (job->mosaic_owners, i), NULL))
      g_ptr_array_add (prints, g_ptr_array_index (job->mosaic_owners, i));
  for (guint i = 0; i < job->owners->len; i++)
    if (!g_ptr_array_find (prints, g_ptr_array_index (job->owners, i), NULL))
      g_ptr_array_add (prints, g_ptr_array_index (job->owners, i));

  for (guint i = 0; i < prints->len; i++)
    {
      FpPrint *print = g_ptr_array_index (prints, i);
      GEnumValue *name = g_enum_get_value (fingers, fp_print_get_finger (print));

      g_string_append_printf (out, "%s%s %.3f", out->len ? ", " : "",
                              name ? name->value_nick : "unknown", print_best (job, print));
    }
  return g_string_free (g_steal_pointer (&out), FALSE);
}

static const char *
finger_name (FpPrint *print)
{
  g_autoptr(GEnumClass) fingers = g_type_class_ref (FP_TYPE_FINGER);
  GEnumValue *name = g_enum_get_value (fingers, fp_print_get_finger (print));

  /* nicks are static strings owned by the enum type */
  return name ? name->value_nick : "unknown";
}

/* Learning into a mosaic print, done in a thread of its own so an unlock
 * never waits for it. Everything it needs is copied, since the print is
 * freed once the unlock completes. */
typedef struct
{
  gchar         *mosaic_path, *library_path;
  const char    *finger;
  E0c7fMosaic   *enrolled; /* to start from if nothing was learned yet */
  GArray        *loose;    /* enrolled presses that never stitched */
  E0c7fTemplate  probe;
} LearnWork;

static GMutex learn_lock; /* one learner at a time per process */
static gboolean learn_in_background = TRUE; /* the tests learn in place */

static void
learn_work_free (LearnWork *w)
{
  g_free (w->mosaic_path);
  g_free (w->library_path);
  e0c7f_mosaic_free (w->enrolled);
  g_array_unref (w->loose);
  g_free (w);
}

static gpointer
learn_work_run (gpointer data)
{
  LearnWork *w = data;
  g_autoptr(GArray) lib = NULL;
  E0c7fMosaic *m;
  E0c7fPlace at;
  guint learned = 0, next, fitted = 0;
  int added = 0;
  gboolean placed;

  g_mutex_lock (&learn_lock);
  /* placed on the mosaic as it is now: another press may have been learned
   * since this one was matched */
  m = load_mosaic_file (w->mosaic_path, &learned);
  if (!m)
    m = e0c7f_mosaic_copy (w->enrolled);
  lib = load_library (w->library_path, &next);
  if (lib->len == 0)
    for (guint i = 0; i < w->loose->len; i++)
      library_add (lib, &next, &g_array_index (w->loose, E0c7fTemplate, i), FALSE);

  placed = e0c7f_mosaic_match (m, &w->probe, E0C7F_MATCH_OVERLAP, &at) >= LEARN_MIN_MOSAIC_SCORE;
  if (placed)
    {
      added = e0c7f_mosaic_add (m, &w->probe, &at);
      learned++;
      /* a grown mosaic may now reach presses that did not fit before */
      if (added > 0)
        fitted = refit_waiting (m, lib);
    }
  library_add (lib, &next, &w->probe, placed);

  if (placed)
    store_mosaic_file (w->mosaic_path, m, learned);
  if (store_library (w->library_path, lib, next))
    {
      if (placed)
        E0C7F_SCORE_LOG ("elan0c7f: learned this press into the %s mosaic (%d new px, "
                         "%d px known, %u presses learned); %u waiting presses fitted in, "
                         "%u still waiting", w->finger, added, e0c7f_mosaic_area (m),
                         learned, fitted, library_waiting (lib));
      else
        E0C7F_SCORE_LOG ("elan0c7f: kept this press for %s to fit in later "
                         "(%u waiting)", w->finger, library_waiting (lib));
    }
  g_mutex_unlock (&learn_lock);
  e0c7f_mosaic_free (m);
  learn_work_free (w);
  return NULL;
}

static void
learn_into_mosaic (MatchJob *job, guint enrolled_idx)
{
  FpPrint *print = g_ptr_array_index (job->mosaic_owners, enrolled_idx);
  LearnWork *w = g_new0 (LearnWork, 1);

  w->mosaic_path = learned_file (print, "mosaic");
  w->library_path = learned_file (print, "presses");
  w->finger = finger_name (print);
  w->enrolled = e0c7f_mosaic_copy (g_ptr_array_index (job->mosaics, enrolled_idx));
  w->probe = job->probe;
  w->loose = g_array_new (FALSE, FALSE, sizeof (E0c7fTemplate));
  for (guint i = 0; i < job->templates->len; i++)
    if (g_ptr_array_index (job->owners, i) == print && !job->learned->data[i])
      g_array_append_vals (w->loose, &g_array_index (job->templates, E0c7fTemplate, i), 1);

  if (learn_in_background)
    g_thread_unref (g_thread_new ("elan0c7f-learn", learn_work_run, w));
  else
    learn_work_run (w);
}

/* after a match: learn the press if it is clearly this finger (see
 * LEARN_DIR for the safeguards). A mosaic print blends it into its learned
 * mosaic; an older print keeps it as a learned template. */
static void
maybe_learn (MatchJob *job)
{
  double enrolled = -1, same = -1, other = -1;
  int target = -1; /* the print's enrolled mosaic, if it has one */
  guint slot;

  if (job->action == FPI_DEVICE_ACTION_ENROLL || !job->probe_ok || !job->scores ||
      !is_match (job))
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
  for (guint i = 0; i < job->mosaics->len; i++)
    {
      if (g_ptr_array_index (job->mosaic_owners, i) != job->best_print)
        other = MAX (other, job->mosaic_scores[i]);
      else if (job->mosaic_learned->data[i])
        same = MAX (same, job->mosaic_scores[i]);
      else
        {
          enrolled = MAX (enrolled, job->mosaic_scores[i]);
          target = i;
        }
    }
  same = MAX (same, enrolled);
  if (enrolled < (target >= 0 ? LEARN_MIN_MOSAIC_SCORE : LEARN_MIN_SCORE) ||
      enrolled - other < LEARN_MIN_MARGIN ||
      job->coverage < LEARN_MIN_COVERAGE || job->clarity < LEARN_MIN_CLARITY)
    return;

  if (target >= 0)
    {
      learn_into_mosaic (job, target);
      return;
    }

  if (same > LEARN_MAX_SCORE)
    return;
  slot = store_learned (job->best_print, &job->probe);
  if (slot == G_MAXUINT)
    return;
  E0C7F_SCORE_LOG ("elan0c7f: learned this press for %s (slot %u of %u)",
                   finger_name (job->best_print), slot + 1, LEARN_SLOTS);
}

/* keep the raw frames of an enrollment press (E0C7F_SAVE_SAMPLES builds),
 * in the layout the offline tools read: press-NN/raw-KKK.bin */
static void
save_sample (FpiDeviceElan0c7f *self, MatchJob *job, guint n)
{
  g_autofree gchar *dir = NULL;

  if (!self->samples_dir)
    return;
  dir = g_strdup_printf ("%s/press-%02u", self->samples_dir, n);
  if (g_mkdir_with_parents (dir, 0700) < 0)
    {
      g_warning ("elan0c7f: could not save enrollment samples in %s", dir);
      return;
    }
  for (int f = 0; f < job->nframes; f++)
    {
      g_autofree gchar *path = g_strdup_printf ("%s/raw-%03d.bin", dir, f);
      g_file_set_contents_full (path, (const gchar *) (job->frames + f * E0C7F_PIXELS),
                                E0C7F_PIXELS * sizeof (guint16), G_FILE_SET_CONTENTS_NONE,
                                0600, NULL);
    }
}

/* and of every unlock press, named by time, finger and score, so misses
 * can be studied later: unlocks/<date>/<time>-<finger>-<score>/raw-KKK.bin */
static void
save_unlock_press (MatchJob *job, double threshold)
{
#ifdef E0C7F_SAVE_SAMPLES
  g_autoptr(GDateTime) now = g_date_time_new_now_local ();
  g_autofree gchar *day = g_date_time_format (now, "%Y%m%d");
  g_autofree gchar *time = g_date_time_format (now, "%H%M%S-%f");
  g_autofree gchar *dir = g_strdup_printf ("%s/unlocks/%s/%s-%s-%.3f-%s", SAMPLES_DIR, day, time,
                                           job->best_print ? finger_name (job->best_print) : "none",
                                           job->best_score,
                                           !job->probe_ok ? "unreadable" :
                                           job->best_score >= threshold ? "match" : "nomatch");

  if (g_mkdir_with_parents (dir, 0700) < 0)
    return;
  for (int f = 0; f < job->nframes; f++)
    {
      g_autofree gchar *path = g_strdup_printf ("%s/raw-%03d.bin", dir, f);
      g_file_set_contents_full (path, (const gchar *) (job->frames + f * E0C7F_PIXELS),
                                E0C7F_PIXELS * sizeof (guint16), G_FILE_SET_CONTENTS_NONE,
                                0600, NULL);
    }
#endif
}

/* and a picture of the finished mosaic: mosaic.pgm, unknown parts black */
static void
save_mosaic_picture (FpiDeviceElan0c7f *self)
{
  const E0c7fMosaic *m = self->enroll_mosaic;
  g_autofree gchar *path = NULL;
  g_autoptr(GByteArray) out = g_byte_array_new ();
  g_autofree gchar *header = NULL;

  if (!self->samples_dir || !m)
    return;
  header = g_strdup_printf ("P5\n%d %d\n255\n", m->w, m->h);
  g_byte_array_append (out, (const guint8 *) header, strlen (header));
  for (int i = 0; i < m->w * m->h; i++)
    {
      guint8 v = m->weight[i] ? MAX (m->img[i], 1) : 0;
      g_byte_array_append (out, &v, 1);
    }
  path = g_build_filename (self->samples_dir, "mosaic.pgm", NULL);
  g_file_set_contents_full (path, (const gchar *) out->data, out->len,
                            G_FILE_SET_CONTENTS_NONE, 0600, NULL);
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
             "%u templates, %u mosaics, best %.3f (threshold %.3f): %s",
             job->action == FPI_DEVICE_ACTION_ENROLL ? "enroll" :
             job->action == FPI_DEVICE_ACTION_VERIFY ? "verify" : "identify",
             job->nframes, job->coverage, job->clarity, job->probe_ok ? "ok" : "rejected",
             job->templates->len, job->mosaics->len,
             job->best_score, match_threshold (job), scores);

  switch (job->action)
    {
    case FPI_DEVICE_ACTION_ENROLL:
      {
        int why = enroll_retry (job);
        gboolean repeat, done;
        guint n, aside = 0;
        int area;

        g_clear_error (&retry);
        /* the thread is done with the enrollment so far: take it back */
        self->enroll_templates = g_steal_pointer (&job->enroll_templates);
        self->enroll_stitched = g_steal_pointer (&job->enroll_stitched);
        self->enroll_mosaic = g_steal_pointer (&job->enroll_mosaic);
        if (why < 0 && job->enroll_too_far)
          {
            E0C7F_SCORE_LOG ("elan0c7f: enroll press too far from the finger so far "
                             "(fit %.3f, needs %.2f): asking again", job->enroll_fit,
                             E0C7F_STITCH_MIN);
            self->enroll_misses++;
            fpi_device_enroll_progress (dev, self->enroll_templates->len, NULL,
                                        fpi_device_retry_new (FP_DEVICE_RETRY_REMOVE_FINGER));
            start_capture (dev);
            return;
          }
        if (why >= 0)
          {
            self->spot_retries++;
            fpi_device_enroll_progress (dev, self->enroll_templates->len, NULL,
                                        fpi_device_retry_new (why));
            start_capture (dev);
            return;
          }
        self->spot_retries = 0;
        self->enroll_misses = 0;
        n = self->enroll_templates->len;
        area = e0c7f_mosaic_area (self->enroll_mosaic);
        for (guint i = 0; i < n; i++)
          aside += !self->enroll_stitched->data[i];
        save_sample (self, job, n);

        /* a press that is stitched on but adds almost nothing is a repeat
         * of a known part; one that does not fit yet may be a new part */
        repeat = job->enroll_fits && job->enroll_added < ENROLL_REPEAT_AREA * E0C7F_PIXELS;
        self->enroll_repeats = repeat ? self->enroll_repeats + 1 : 0;
        done = n >= ENROLL_MAX_PRESSES ||
               (n >= ENROLL_MIN_PRESSES && self->enroll_repeats >= ENROLL_REPEATS &&
                area >= ENROLL_MIN_AREA * E0C7F_PIXELS && aside <= ENROLL_MAX_ASIDE);
        E0C7F_SCORE_LOG ("elan0c7f: enroll press %u: %s, %d new px, mosaic %d px "
                         "(%.1f presses' worth), %u aside, %u repeats in a row%s",
                         n, job->enroll_fits ? "stitched" : "kept aside", job->enroll_added,
                         area, (double) area / E0C7F_PIXELS, aside, self->enroll_repeats,
                         done ? ": finger covered, done" : "");
        fpi_device_enroll_progress (dev, n, NULL, NULL);
        if (!done)
          {
            start_capture (dev);
            return;
          }
        fpi_device_get_enroll_data (dev, &print);
        fpi_print_set_type (print, FPI_PRINT_RAW);
        g_object_set (print, "fpi-data",
                      mosaic_print_to_variant (self->enroll_templates, self->enroll_stitched,
                                               self->enroll_mosaic), NULL);
        save_mosaic_picture (self);
        enroll_state_clear (self);
        fpi_device_enroll_complete (dev, g_object_ref (print), NULL);
      }
      break;

    case FPI_DEVICE_ACTION_VERIFY:
      /* before reporting: the prints are freed once the action completes */
      save_unlock_press (job, match_threshold (job));
      maybe_learn (job);
      if (retry)
        fpi_device_verify_report (dev, FPI_MATCH_ERROR, NULL, retry);
      else
        fpi_device_verify_report (dev,
                                  is_match (job) ? FPI_MATCH_SUCCESS : FPI_MATCH_FAIL,
                                  NULL, NULL);
      fpi_device_verify_complete (dev, NULL);
      break;

    case FPI_DEVICE_ACTION_IDENTIFY:
      /* before reporting: the prints are freed once the action completes */
      save_unlock_press (job, match_threshold (job));
      maybe_learn (job);
      if (retry)
        fpi_device_identify_report (dev, NULL, NULL, retry);
      else
        fpi_device_identify_report (dev,
                                    is_match (job) ? job->best_print : NULL,
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
  job->mosaics = g_ptr_array_new_with_free_func ((GDestroyNotify) e0c7f_mosaic_free);
  job->mosaic_owners = g_ptr_array_new ();
  job->mosaic_learned = g_byte_array_new ();

  if (action == FPI_DEVICE_ACTION_ENROLL)
    {
      FpiDeviceElan0c7f *self = FPI_DEVICE_ELAN0C7F (dev);

      /* lent to the thread, which stitches the press on; report_match()
       * takes them back */
      job->enroll_templates = g_steal_pointer (&self->enroll_templates);
      job->enroll_stitched = g_steal_pointer (&self->enroll_stitched);
      job->enroll_mosaic = g_steal_pointer (&self->enroll_mosaic);
      job->spot_retries = self->spot_retries;
      job->enroll_misses = self->enroll_misses;
    }
  else if (action == FPI_DEVICE_ACTION_VERIFY)
    {
      FpPrint *print;

      fpi_device_get_verify_data (dev, &print);
      if (!add_print_templates (print, job))
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
        add_print_templates (g_ptr_array_index (prints, i), job);
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
      enroll_state_clear (self);
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

  enroll_state_clear (self);
  self->enroll_templates = g_array_new (FALSE, FALSE, sizeof (E0c7fTemplate));
  self->enroll_stitched = g_byte_array_new ();
  self->enroll_repeats = 0;
  self->enroll_misses = 0;
  self->spot_retries = 0;
#ifdef E0C7F_SAVE_SAMPLES
  {
    g_autoptr(GDateTime) now = g_date_time_new_now_local ();
    g_autofree gchar *stamp = g_date_time_format (now, "%Y%m%d-%H%M%S");
    FpPrint *print;

    fpi_device_get_enroll_data (dev, &print);
    self->samples_dir = g_strdup_printf ("%s/%s-%s", SAMPLES_DIR, finger_name (print), stamp);
  }
#endif
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
