/*
 * Image processing and matching for the ELAN 04f3:0c7f press sensor
 *
 * The sensor only sees a 52x150 patch of the finger, which is too small for
 * minutiae matching. Instead the ridge pattern is cleaned up and compared by
 * normalised cross-correlation over small rotations and shifts.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#pragma once

#include <stdint.h>

/* raw frame layout: 150 rows of 52 pixels, 16-bit little endian */
#define E0C7F_W 52
#define E0C7F_H 150
#define E0C7F_PIXELS (E0C7F_W * E0C7F_H)

/* scores at or above this are a match. With guided enrollment, live
 * wrong-finger scores stayed at or below 0.40 while right fingers scored
 * 0.46-0.73 (one older recorded sample reached 0.51) */
#define E0C7F_MATCH_THRESHOLD 0.50
/* prints with a mosaic: wrong fingers search a larger image there, and a
 * 20-press mosaic let one recorded wrong-finger press reach 0.50, while
 * right fingers scored 0.60 or more against it */
#define E0C7F_MOSAIC_THRESHOLD 0.55

typedef struct
{
  uint8_t img[E0C7F_PIXELS];  /* cleaned ridge image */
  /* 0 where the finger did not cover the sensor; elsewhere the local ridge
   * clarity, 2..255 for 0..1, or 1 where unknown (flat image, or a template
   * made before clarity was kept) */
  uint8_t mask[E0C7F_PIXELS];
} E0c7fTemplate;


/* Build a template from consecutive raw frames of one press.
 * Returns 0 on success, -1 if the finger covered too little of the sensor. */
int e0c7f_template_from_frames (const uint16_t *frames,
                                int             nframes,
                                E0c7fTemplate  *out);

/* Image quality of a template: fraction of the sensor covered by the finger
 * and ridge clarity (mean orientation coherence, 0 = noise, 1 = clean
 * parallel ridges). */
void e0c7f_template_quality (const E0c7fTemplate *t,
                             double              *coverage,
                             double              *clarity);

/* Similarity of a probe to an enrolled template, in [-1, 1]. */
double e0c7f_match (const E0c7fTemplate *enrolled,
                    const E0c7fTemplate *probe);

/* ---- mosaic -------------------------------------------------------------
 * A mosaic stitches many presses of one finger into one larger ridge image,
 * so a press is judged against the whole known finger instead of against
 * each stored press separately. Presses are placed by the same rotation and
 * shift search as e0c7f_match() and averaged where they overlap, weighted
 * towards each press's centre. */

typedef struct
{
  int      w, h;
  uint8_t *img;    /* ridge values */
  uint8_t *weight; /* how well each pixel is known: E0C7F_PRESS_WEIGHT per
                    * press at its centre, capped; 0 = unknown */
} E0c7fMosaic;

/* weight one press gives at its centre, and the cap, so that once a pixel
 * has been seen E0C7F_WEIGHT_CAP / E0C7F_PRESS_WEIGHT times every new press
 * still moves it by a fixed share (the mosaic keeps adapting) */
#define E0C7F_PRESS_WEIGHT 16
#define E0C7F_WEIGHT_CAP   128

typedef struct
{
  double score;
  int    angle, dx, dy; /* rotation of the press, then its offset on the mosaic */
} E0c7fPlace;

E0c7fMosaic *e0c7f_mosaic_new (const E0c7fTemplate *first);
E0c7fMosaic *e0c7f_mosaic_copy (const E0c7fMosaic *m);
/* takes ownership of img and weight (w * h bytes each, from malloc) */
E0c7fMosaic *e0c7f_mosaic_wrap (int w, int h, uint8_t *img, uint8_t *weight);
void         e0c7f_mosaic_free (E0c7fMosaic *m);

/* a press must overlap the known finger by this share of its own area.
 * Matching asks for more: the mosaic should know most of the finger, and
 * small overlaps are where a wrong finger finds lucky matches in a large
 * image. Stitching asks for less, so a press can extend the edge. */
#define E0C7F_MATCH_OVERLAP  0.50
#define E0C7F_STITCH_OVERLAP 0.35
/* a press is stitched on if it fits this well; below, it is kept aside and
 * tried again once the mosaic has grown. Presses of different fingers
 * stayed at or below 0.47 against each other (2700 recorded pairs) */
#define E0C7F_STITCH_MIN 0.50

/* Best placement of probe on the mosaic; returns its score in [-1, 1]
 * (-1 if nothing overlaps enough). where may be NULL. */
double e0c7f_mosaic_match (const E0c7fMosaic   *m,
                           const E0c7fTemplate *probe,
                           double               min_overlap,
                           E0c7fPlace          *where);

/* Blend probe into the mosaic at where, growing it as needed. Returns how
 * many pixels were unknown before (the new area this press added). */
int e0c7f_mosaic_add (E0c7fMosaic         *m,
                      const E0c7fTemplate *probe,
                      const E0c7fPlace    *where);

/* number of known pixels */
int e0c7f_mosaic_area (const E0c7fMosaic *m);
