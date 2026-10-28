/*
 * Image processing and matching for the ELAN 04f3:0c7f press sensor
 *
 * Copyright (C) 2026 Kumarsujay03
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
 * ridge detail to 0..255. */
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
  return 0;
}

void
e0c7f_template_quality (const E0c7fTemplate *t,
                        double              *coverage,
                        double              *clarity)
{
  float *gxx = calloc (N, sizeof (float)), *gyy = calloc (N, sizeof (float));
  float *gxy = calloc (N, sizeof (float));
  float *sxx = malloc (N * sizeof (float)), *syy = malloc (N * sizeof (float));
  float *sxy = malloc (N * sizeof (float));
  double sum = 0;
  int covered = 0, counted = 0;

  for (int i = 0; i < N; i++)
    covered += t->mask[i];
  *coverage = (double) covered / N;

  /* structure tensor from central differences inside the finger area */
  for (int y = 1; y < H - 1; y++)
    for (int x = 1; x < W - 1; x++)
      {
        int i = y * W + x;
        if (!t->mask[i - 1] || !t->mask[i + 1] || !t->mask[i - W] || !t->mask[i + W])
          continue;
        float gx = (t->img[i + 1] - t->img[i - 1]) / 2.0f;
        float gy = (t->img[i + W] - t->img[i - W]) / 2.0f;
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
      if (!t->mask[i] || energy < 1e-3)
        continue;
      sum += sqrt ((sxx[i] - syy[i]) * (sxx[i] - syy[i]) + 4 * sxy[i] * sxy[i]) / energy;
      counted++;
    }
  *clarity = counted ? sum / counted : 0;

  free (gxx);
  free (gyy);
  free (gxy);
  free (sxx);
  free (syy);
  free (sxy);
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
