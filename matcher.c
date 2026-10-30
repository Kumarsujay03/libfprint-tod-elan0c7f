/*
 * Image processing and matching for the ELAN 04f3:0c7f press sensor
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include "matcher.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#define W E0C7F_W
#define H E0C7F_H
#define N E0C7F_PIXELS

/* frames covering less than this fraction of the sensor are not a press */
#define MIN_COVERAGE 0.3
/* keep frames within this fraction of the best coverage (drops touch/lift) */
#define STABLE_COVERAGE 0.9
/* shading removal blur, in pixels */
#define SHADING_SIGMA 6.0
/* placements with less overlap than this fraction of the template are skipped */
#define MIN_OVERLAP 0.35
#define MAX_ANGLE 24
#define ANGLE_STEP 4
#define SHIFT_STEP 2
/* search a sparse grid first, then refine around the best placements */
#define COARSE_ANGLE_STEP 8
#define COARSE_SHIFT_STEP 4
#define REFINE_CANDIDATES 3
/* smoothing for the ridge orientation used by the quality check */
#define CLARITY_SIGMA 3.0

static int
cmp_float (const void *a, const void *b)
{
  float fa = *(const float *) a, fb = *(const float *) b;

  return (fa > fb) - (fa < fb);
}

/* median of n values (sorts in place); mean of the middle pair for even n */
static float
median (float *v, int n)
{
  qsort (v, n, sizeof (float), cmp_float);
  return (n % 2) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

/* linear-interpolated percentile of sorted values */
static float
percentile (const float *sorted, int n, double p)
{
  double idx = p / 100.0 * (n - 1);
  int lo = (int) floor (idx);
  int hi = lo + 1 < n ? lo + 1 : lo;

  return sorted[lo] + (float) (idx - lo) * (sorted[hi] - sorted[lo]);
}

static int
reflect101 (int i, int n)
{
  while (i < 0 || i >= n)
    i = i < 0 ? -i : 2 * (n - 1) - i;
  return i;
}

/* separable Gaussian blur, reflect-101 borders */
static void
gaussian_blur (const float *src, float *dst, double sigma)
{
  int r = (int) lround (sigma * 4);
  int k = 2 * r + 1;
  float *kern = malloc (k * sizeof (float));
  float *tmp = malloc (N * sizeof (float));
  double sum = 0;

  for (int i = 0; i < k; i++)
    sum += kern[i] = expf (-(float) ((i - r) * (i - r)) / (2 * sigma * sigma));
  for (int i = 0; i < k; i++)
    kern[i] /= sum;

  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      {
        float acc = 0;
        for (int i = 0; i < k; i++)
          acc += kern[i] * src[y * W + reflect101 (x + i - r, W)];
        tmp[y * W + x] = acc;
      }
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      {
        float acc = 0;
        for (int i = 0; i < k; i++)
          acc += kern[i] * tmp[reflect101 (y + i - r, H) * W + x];
        dst[y * W + x] = acc;
      }

  free (tmp);
  free (kern);
}

/* Remove per-row and per-column offsets and slow shading, then stretch the
 * ridge detail to 0..255. Mirrors what the Windows driver does. */
static void
clean_image (float *img, const uint8_t *mask, uint8_t *out)
{
  float *vals = malloc (N * sizeof (float));
  float *bg = malloc (N * sizeof (float));
  int n;

  for (int y = 0; y < H; y++)
    {
      n = 0;
      for (int x = 0; x < W; x++)
        if (mask[y * W + x])
          vals[n++] = img[y * W + x];
      if (n)
        {
          float m = median (vals, n);
          for (int x = 0; x < W; x++)
            img[y * W + x] -= m;
        }
    }
  for (int x = 0; x < W; x++)
    {
      n = 0;
      for (int y = 0; y < H; y++)
        if (mask[y * W + x])
          vals[n++] = img[y * W + x];
      if (n)
        {
          float m = median (vals, n);
          for (int y = 0; y < H; y++)
            img[y * W + x] -= m;
        }
    }

  gaussian_blur (img, bg, SHADING_SIGMA);
  n = 0;
  for (int i = 0; i < N; i++)
    {
      img[i] = mask[i] ? img[i] - bg[i] : 0;
      if (mask[i])
        vals[n++] = img[i];
    }

  float lo = 0, hi = 1;
  if (n)
    {
      qsort (vals, n, sizeof (float), cmp_float);
      lo = percentile (vals, n, 2);
      hi = percentile (vals, n, 98);
    }
  float range = hi - lo > 1e-6f ? hi - lo : 1e-6f;
  for (int i = 0; i < N; i++)
    {
      float v = mask[i] ? (img[i] - lo) / range : 0;
      v = v < 0 ? 0 : v > 1 ? 1 : v;
      out[i] = mask[i] ? (uint8_t) (v * 255) : 0;
    }

  free (bg);
  free (vals);
}

/* how clean the ridges of a cleaned image are at each pixel: orientation
 * coherence, 0 = noise .. 1 = clean parallel ridges; -1 where the image is
 * flat or not covered */
static void
ridge_clarity (const uint8_t *img, const uint8_t *mask, float *coherence)
{
  float *gxx = calloc (N, sizeof (float)), *gyy = calloc (N, sizeof (float));
  float *gxy = calloc (N, sizeof (float));
  float *sxx = malloc (N * sizeof (float)), *syy = malloc (N * sizeof (float));
  float *sxy = malloc (N * sizeof (float));

  /* structure tensor from central differences inside the finger area */
  for (int y = 1; y < H - 1; y++)
    for (int x = 1; x < W - 1; x++)
      {
        int i = y * W + x;
        if (!mask[i - 1] || !mask[i + 1] || !mask[i - W] || !mask[i + W])
          continue;
        float gx = (img[i + 1] - img[i - 1]) / 2.0f;
        float gy = (img[i + W] - img[i - W]) / 2.0f;
        gxx[i] = gx * gx;
        gyy[i] = gy * gy;
        gxy[i] = gx * gy;
      }
  gaussian_blur (gxx, sxx, CLARITY_SIGMA);
  gaussian_blur (gyy, syy, CLARITY_SIGMA);
  gaussian_blur (gxy, sxy, CLARITY_SIGMA);

  for (int i = 0; i < N; i++)
    {
      double energy = sxx[i] + syy[i];
      coherence[i] = -1;
      if (!mask[i] || energy < 1e-3)
        continue;
      coherence[i] = sqrt ((sxx[i] - syy[i]) * (sxx[i] - syy[i]) + 4 * sxy[i] * sxy[i]) / energy;
    }

  free (gxx);
  free (gyy);
  free (gxy);
  free (sxx);
  free (syy);
  free (sxy);
}

int
e0c7f_template_from_frames (const uint16_t *frames,
                            int             nframes,
                            E0c7fTemplate  *out)
{
  double *coverage = malloc (nframes * sizeof (double));
  double best = 0;
  float *avg = calloc (N, sizeof (float));
  int used = 0;

  for (int f = 0; f < nframes; f++)
    {
      int c = 0;
      for (int i = 0; i < N; i++)
        c += frames[f * N + i] > 0;
      coverage[f] = (double) c / N;
      if (coverage[f] > best)
        best = coverage[f];
    }

  memset (out->mask, 1, N);
  for (int f = 0; f < nframes; f++)
    {
      if (coverage[f] < STABLE_COVERAGE * best || coverage[f] <= MIN_COVERAGE)
        continue;
      for (int i = 0; i < N; i++)
        {
          avg[i] += frames[f * N + i];
          out->mask[i] &= frames[f * N + i] > 0;
        }
      used++;
    }
  free (coverage);

  if (!used)
    {
      free (avg);
      return -1;
    }
  for (int i = 0; i < N; i++)
    avg[i] /= used;

  clean_image (avg, out->mask, out->img);
  free (avg);

  /* keep each pixel's clarity in the mask (see E0c7fTemplate) */
  float *coherence = malloc (N * sizeof (float));
  ridge_clarity (out->img, out->mask, coherence);
  for (int i = 0; i < N; i++)
    if (out->mask[i])
      out->mask[i] = coherence[i] < 0 ? 1 : 2 + (int) lround (coherence[i] * 253);
  free (coherence);
  return 0;
}

void
e0c7f_template_quality (const E0c7fTemplate *t,
                        double              *coverage,
                        double              *clarity)
{
  double sum = 0;
  int covered = 0, counted = 0;

  /* the mask holds each pixel's clarity */
  for (int i = 0; i < N; i++)
    {
      covered += t->mask[i] != 0;
      if (t->mask[i] > 1)
        {
          sum += t->mask[i] / 255.0;
          counted++;
        }
    }
  *coverage = (double) covered / N;
  *clarity = counted ? sum / counted : 0;
}

/* template as float with an eroded mask; invalid pixels hold -1 */
static void
ridge_values (const E0c7fTemplate *t, float *vals, uint8_t *valid)
{
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      {
        int ok = 1;
        /* 3x3 erosion; pixels outside the image do not erode */
        for (int dy = -1; dy <= 1 && ok; dy++)
          for (int dx = -1; dx <= 1 && ok; dx++)
            {
              int yy = y + dy, xx = x + dx;
              if (yy >= 0 && yy < H && xx >= 0 && xx < W && !t->mask[yy * W + xx])
                ok = 0;
            }
        valid[y * W + x] = ok;
        vals[y * W + x] = ok ? t->img[y * W + x] : -1;
      }
}

/* rotate about the image centre with bilinear sampling; a result is only
 * valid if it is not pulled below zero by invalid (-1) neighbours */
static void
rotate (const float *src, float *dst, uint8_t *valid, double angle_deg)
{
  double a = angle_deg * M_PI / 180.0;
  double c = cos (a), s = sin (a);
  double cx = W / 2.0, cy = H / 2.0;

  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      {
        double sx = c * (x - cx) - s * (y - cy) + cx;
        double sy = s * (x - cx) + c * (y - cy) + cy;
        int x0 = (int) floor (sx), y0 = (int) floor (sy);
        double fx = sx - x0, fy = sy - y0;
        double v = 0;

        for (int j = 0; j < 2; j++)
          for (int i = 0; i < 2; i++)
            {
              int xx = x0 + i, yy = y0 + j;
              double w = (i ? fx : 1 - fx) * (j ? fy : 1 - fy);
              double p = (xx >= 0 && xx < W && yy >= 0 && yy < H) ? src[yy * W + xx] : -1;
              v += w * p;
            }
        dst[y * W + x] = (float) v;
        valid[y * W + x] = v >= 0;
      }
}

/* NCC of the rotated probe shifted by (dx, dy) over the enrolled template,
 * or -1 if the overlap is too small */
static double
score_at (const float *a, const uint8_t *am, int a_count,
          const float *rb, const uint8_t *rbm, int dx, int dy)
{
  int ys = dy > 0 ? dy : 0, ye = dy < 0 ? H + dy : H;
  int xs = dx > 0 ? dx : 0, xe = dx < 0 ? W + dx : W;
  double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
  int n = 0;

  for (int y = ys; y < ye; y++)
    {
      const float *pa = a + y * W;
      const uint8_t *pam = am + y * W;
      const float *pb = rb + (y - dy) * W - dx;
      const uint8_t *pbm = rbm + (y - dy) * W - dx;
      for (int x = xs; x < xe; x++)
        if (pam[x] && pbm[x])
          {
            double va = pa[x], vb = pb[x];
            sa += va;
            sb += vb;
            saa += va * va;
            sbb += vb * vb;
            sab += va * vb;
            n++;
          }
    }
  if (n < MIN_OVERLAP * a_count)
    return -1;
  double d = (saa - sa * sa / n) * (sbb - sb * sb / n);
  return d > 0 ? (sab - sa * sb / n) / sqrt (d) : -1;
}

typedef struct
{
  double score;
  int    angle, dx, dy;
} Placement;

/* keep the best few coarse placements, highest first */
static void
keep_best (Placement *best, int nbest, Placement p)
{
  for (int i = 0; i < nbest; i++)
    if (p.score > best[i].score)
      {
        memmove (best + i + 1, best + i, (nbest - i - 1) * sizeof (Placement));
        best[i] = p;
        return;
      }
}

double
e0c7f_match (const E0c7fTemplate *enrolled,
             const E0c7fTemplate *probe)
{
  /* on the stack so matches can run in parallel threads */
  float a[N], b[N], rb[N];
  uint8_t am[N], bm[N], rbm[N];
  Placement coarse[REFINE_CANDIDATES];
  double best = -1;
  int a_count = 0;

  ridge_values (enrolled, a, am);
  ridge_values (probe, b, bm);
  for (int i = 0; i < N; i++)
    a_count += am[i];
  for (int i = 0; i < REFINE_CANDIDATES; i++)
    coarse[i] = (Placement) { .score = -2 };

  /* coarse search over a sparse grid of rotations and shifts */
  for (int ang = -MAX_ANGLE; ang <= MAX_ANGLE; ang += COARSE_ANGLE_STEP)
    {
      rotate (b, rb, rbm, ang);
      for (int dy = -H / 2; dy <= H / 2; dy += COARSE_SHIFT_STEP)
        for (int dx = -W / 3; dx <= W / 3; dx += COARSE_SHIFT_STEP)
          keep_best (coarse, REFINE_CANDIDATES,
                     (Placement) { score_at (a, am, a_count, rb, rbm, dx, dy), ang, dx, dy });
    }

  /* refine around the best coarse placements on the fine grid */
  for (int c = 0; c < REFINE_CANDIDATES && coarse[c].score > -2; c++)
    for (int ang = coarse[c].angle - COARSE_ANGLE_STEP + ANGLE_STEP;
         ang < coarse[c].angle + COARSE_ANGLE_STEP; ang += ANGLE_STEP)
      {
        rotate (b, rb, rbm, ang);
        for (int dy = coarse[c].dy - COARSE_SHIFT_STEP + SHIFT_STEP;
             dy < coarse[c].dy + COARSE_SHIFT_STEP; dy += SHIFT_STEP)
          for (int dx = coarse[c].dx - COARSE_SHIFT_STEP + SHIFT_STEP;
               dx < coarse[c].dx + COARSE_SHIFT_STEP; dx += SHIFT_STEP)
            {
              double s = score_at (a, am, a_count, rb, rbm, dx, dy);
              if (s > best)
                best = s;
            }
      }
  return best;
}

/* ---- mosaic ------------------------------------------------------------ */

/* pixels weighted below this are only the soft edge of a press */
#define MIN_KNOWN 4
/* press weight ramps up over this many pixels from its edge */
#define FEATHER 6
/* spare border kept around the known area when the mosaic grows */
#define GROW_PAD 16
#define MOSAIC_CANDIDATES 5

E0c7fMosaic *
e0c7f_mosaic_wrap (int w, int h, uint8_t *img, uint8_t *weight)
{
  E0c7fMosaic *m = malloc (sizeof *m);

  *m = (E0c7fMosaic) { w, h, img, weight };
  return m;
}

static E0c7fMosaic *
mosaic_blank (int w, int h)
{
  return e0c7f_mosaic_wrap (w, h, calloc (w * h, 1), calloc (w * h, 1));
}

E0c7fMosaic *
e0c7f_mosaic_copy (const E0c7fMosaic *m)
{
  E0c7fMosaic *c = mosaic_blank (m->w, m->h);

  memcpy (c->img, m->img, m->w * m->h);
  memcpy (c->weight, m->weight, m->w * m->h);
  return c;
}

void
e0c7f_mosaic_free (E0c7fMosaic *m)
{
  if (!m)
    return;
  free (m->img);
  free (m->weight);
  free (m);
}

E0c7fMosaic *
e0c7f_mosaic_new (const E0c7fTemplate *first)
{
  E0c7fMosaic *m = mosaic_blank (W + 2 * GROW_PAD, H + 2 * GROW_PAD);
  E0c7fPlace at = { 1, 0, GROW_PAD, GROW_PAD };

  e0c7f_mosaic_add (m, first, &at);
  return m;
}

int
e0c7f_mosaic_area (const E0c7fMosaic *m)
{
  int n = 0;

  for (int i = 0; i < m->w * m->h; i++)
    n += m->weight[i] >= MIN_KNOWN;
  return n;
}

/* the mosaic as float ridge values with an eroded "known" mask, as
 * ridge_values() does for a template */
static void
mosaic_values (const E0c7fMosaic *m, float *vals, uint8_t *valid)
{
  for (int y = 0; y < m->h; y++)
    for (int x = 0; x < m->w; x++)
      {
        int ok = 1;
        for (int dy = -1; dy <= 1 && ok; dy++)
          for (int dx = -1; dx <= 1 && ok; dx++)
            {
              int yy = y + dy, xx = x + dx;
              if (yy < 0 || yy >= m->h || xx < 0 || xx >= m->w ||
                  m->weight[yy * m->w + xx] < MIN_KNOWN)
                ok = 0;
            }
        valid[y * m->w + x] = ok;
        vals[y * m->w + x] = ok ? m->img[y * m->w + x] : -1;
      }
}

/* NCC of the rotated probe placed with its top-left at (dx, dy) on the
 * mosaic, over every stride-th row and column; -1 if fewer than min_count
 * pixels overlap */
static double
score_on (const float *a, const uint8_t *am, int aw, int ah,
          const float *rb, const uint8_t *rbm, int min_count,
          int dx, int dy, int stride)
{
  int ys = dy < 0 ? -dy : 0, ye = ah - dy < H ? ah - dy : H;
  int xs = dx < 0 ? -dx : 0, xe = aw - dx < W ? aw - dx : W;
  double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
  int n = 0;

  for (int y = ys; y < ye; y += stride)
    {
      const float *pa = a + (y + dy) * aw + dx;
      const uint8_t *pam = am + (y + dy) * aw + dx;
      const float *pb = rb + y * W;
      const uint8_t *pbm = rbm + y * W;
      for (int x = xs; x < xe; x += stride)
        if (pam[x] && pbm[x])
          {
            double va = pa[x], vb = pb[x];
            sa += va;
            sb += vb;
            saa += va * va;
            sbb += vb * vb;
            sab += va * vb;
            n++;
          }
    }
  if (n * stride * stride < min_count)
    return -1;
  double d = (saa - sa * sa / n) * (sbb - sb * sb / n);
  return d > 0 ? (sab - sa * sb / n) / sqrt (d) : -1;
}

double
e0c7f_mosaic_match (const E0c7fMosaic   *m,
                    const E0c7fTemplate *probe,
                    double               min_overlap,
                    E0c7fPlace          *where)
{
  int mn = m->w * m->h;
  float *a = malloc (mn * sizeof (float));
  uint8_t *am = malloc (mn);
  float b[N], rb[N];
  uint8_t bm[N], rbm[N];
  Placement coarse[MOSAIC_CANDIDATES];
  E0c7fPlace best = { -1, 0, 0, 0 };
  int min_count = 0;

  mosaic_values (m, a, am);
  ridge_values (probe, b, bm);
  for (int i = 0; i < N; i++)
    min_count += bm[i];
  min_count = (int) (min_overlap * min_count);
  for (int i = 0; i < MOSAIC_CANDIDATES; i++)
    coarse[i] = (Placement) { .score = -2 };

  /* coarse: sparse rotations and shifts, every other pixel; the probe's
   * centre stays on the mosaic */
  for (int ang = -MAX_ANGLE; ang <= MAX_ANGLE; ang += COARSE_ANGLE_STEP)
    {
      rotate (b, rb, rbm, ang);
      for (int dy = -H / 2; dy <= m->h - H / 2; dy += COARSE_SHIFT_STEP)
        for (int dx = -W / 2; dx <= m->w - W / 2; dx += COARSE_SHIFT_STEP)
          keep_best (coarse, MOSAIC_CANDIDATES,
                     (Placement) { score_on (a, am, m->w, m->h, rb, rbm, min_count, dx, dy, 2),
                                   ang, dx, dy });
    }

  for (int c = 0; c < MOSAIC_CANDIDATES && coarse[c].score > -2; c++)
    for (int ang = coarse[c].angle - COARSE_ANGLE_STEP + ANGLE_STEP;
         ang < coarse[c].angle + COARSE_ANGLE_STEP; ang += ANGLE_STEP)
      {
        rotate (b, rb, rbm, ang);
        for (int dy = coarse[c].dy - COARSE_SHIFT_STEP + 1; dy < coarse[c].dy + COARSE_SHIFT_STEP; dy++)
          for (int dx = coarse[c].dx - COARSE_SHIFT_STEP + 1; dx < coarse[c].dx + COARSE_SHIFT_STEP; dx++)
            {
              double s = score_on (a, am, m->w, m->h, rb, rbm, min_count, dx, dy, 1);
              if (s > best.score)
                best = (E0c7fPlace) { s, ang, dx, dy };
            }
      }

  free (a);
  free (am);
  if (where)
    *where = best;
  return best.score;
}

/* where presses overlap, clearer ridges count for more: a pixel's weight
 * is scaled by its clarity relative to TYPICAL_CLARITY, within these bounds */
#define TYPICAL_CLARITY 0.35
#define MIN_CLARITY_SCALE 0.25
#define MAX_CLARITY_SCALE 2.0

/* per pixel weight of a press: E0C7F_PRESS_WEIGHT inside, ramping down to
 * 1 at the edge of its (eroded) mask, scaled by clarity; 0 outside */
static void
press_weights (const uint8_t *valid, const uint8_t *clarity, float *wt)
{
  /* city-block distance to the nearest invalid pixel or the border */
  int *d = malloc (N * sizeof (int));

  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      {
        int i = y * W + x, v = valid[i] ? 1 : 0;
        if (v)
          {
            int up = y ? d[i - W] : 0, left = x ? d[i - 1] : 0;
            v = 1 + (up < left ? up : left);
          }
        d[i] = v;
      }
  for (int y = H - 1; y >= 0; y--)
    for (int x = W - 1; x >= 0; x--)
      {
        int i = y * W + x;
        if (!d[i])
          continue;
        int down = y < H - 1 ? d[i + W] + 1 : 1, right = x < W - 1 ? d[i + 1] + 1 : 1;
        if (down < d[i])
          d[i] = down;
        if (right < d[i])
          d[i] = right;
      }
  for (int i = 0; i < N; i++)
    {
      double scale = 1;
      if (clarity[i] > 1)
        {
          scale = (clarity[i] - 2) / 253.0 / TYPICAL_CLARITY;
          scale = scale < MIN_CLARITY_SCALE ? MIN_CLARITY_SCALE :
                  scale > MAX_CLARITY_SCALE ? MAX_CLARITY_SCALE : scale;
        }
      wt[i] = d[i] ? E0C7F_PRESS_WEIGHT * scale * (d[i] < FEATHER ? d[i] : FEATHER) / FEATHER : 0;
    }
  free (d);
}

/* grow m so that the box [x0, x1) x [y0, y1) fits, with GROW_PAD to spare;
 * returns how far the old content moved */
static void
mosaic_fit (E0c7fMosaic *m, int x0, int y0, int x1, int y1, int *ox, int *oy)
{
  int left = x0 < 0 ? GROW_PAD - x0 : 0, top = y0 < 0 ? GROW_PAD - y0 : 0;
  int right = x1 > m->w ? x1 - m->w + GROW_PAD : 0, bottom = y1 > m->h ? y1 - m->h + GROW_PAD : 0;

  *ox = left;
  *oy = top;
  if (!left && !top && !right && !bottom)
    return;

  int nw = m->w + left + right, nh = m->h + top + bottom;
  uint8_t *img = calloc (nw * nh, 1), *weight = calloc (nw * nh, 1);
  for (int y = 0; y < m->h; y++)
    {
      memcpy (img + (y + top) * nw + left, m->img + y * m->w, m->w);
      memcpy (weight + (y + top) * nw + left, m->weight + y * m->w, m->w);
    }
  free (m->img);
  free (m->weight);
  m->img = img;
  m->weight = weight;
  m->w = nw;
  m->h = nh;
}

int
e0c7f_mosaic_add (E0c7fMosaic *m, const E0c7fTemplate *probe, const E0c7fPlace *where)
{
  float vals[N], wt[N];
  uint8_t valid[N];
  double a = where->angle * M_PI / 180.0, c = cos (a), s = sin (a);
  double cx = W / 2.0, cy = H / 2.0;
  int x0 = INT32_MAX, y0 = INT32_MAX, x1 = INT32_MIN, y1 = INT32_MIN, ox, oy, added = 0;

  ridge_values (probe, vals, valid);
  press_weights (valid, probe->mask, wt);

  /* rotate() samples source pixel R (p - c) + c for frame pixel p, and the
   * frame sits at (dx, dy): so a source corner lands at R^T (q - c) + c + d */
  for (int k = 0; k < 4; k++)
    {
      double qx = (k & 1) ? W - 1 : 0, qy = (k & 2) ? H - 1 : 0;
      double px = c * (qx - cx) + s * (qy - cy) + cx + where->dx;
      double py = -s * (qx - cx) + c * (qy - cy) + cy + where->dy;
      x0 = MIN (x0, (int) floor (px));
      y0 = MIN (y0, (int) floor (py));
      x1 = MAX (x1, (int) ceil (px) + 1);
      y1 = MAX (y1, (int) ceil (py) + 1);
    }
  mosaic_fit (m, x0, y0, x1, y1, &ox, &oy);
  x0 += ox; x1 += ox; y0 += oy; y1 += oy;

  for (int y = y0; y < y1; y++)
    for (int x = x0; x < x1; x++)
      {
        double px = x - where->dx - ox, py = y - where->dy - oy;
        double sx = c * (px - cx) - s * (py - cy) + cx;
        double sy = s * (px - cx) + c * (py - cy) + cy;
        int xi = (int) floor (sx), yi = (int) floor (sy);
        double fx = sx - xi, fy = sy - yi, v = 0, w = 0;

        if (xi < 0 || yi < 0 || xi + 1 >= W || yi + 1 >= H)
          continue;
        int i = yi * W + xi;
        if (!valid[i] || !valid[i + 1] || !valid[i + W] || !valid[i + W + 1])
          continue;
        v = (1 - fy) * ((1 - fx) * vals[i] + fx * vals[i + 1]) + fy * ((1 - fx) * vals[i + W] + fx * vals[i + W + 1]);
        w = (1 - fy) * ((1 - fx) * wt[i] + fx * wt[i + 1]) + fy * ((1 - fx) * wt[i + W] + fx * wt[i + W + 1]);
        int wi = (int) lround (w);
        if (wi < 1)
          wi = 1;

        int j = y * m->w + x, old = m->weight[j];
        int nw = old + wi;
        m->img[j] = (uint8_t) lround ((m->img[j] * (double) old + v * wi) / nw);
        m->weight[j] = nw > E0C7F_WEIGHT_CAP ? E0C7F_WEIGHT_CAP : nw;
        added += old < MIN_KNOWN && m->weight[j] >= MIN_KNOWN;
      }
  return added;
}
