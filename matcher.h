/*
 * Image processing and matching for the ELAN 04f3:0c7f press sensor
 *
 * The sensor only sees a 52x150 patch of the finger, which is too small for
 * minutiae matching. Instead the ridge pattern is cleaned up and compared by
 * normalised cross-correlation over small rotations and shifts.
 *
 * Copyright (C) 2026 Kumarsujay03
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

typedef struct
{
  uint8_t img[E0C7F_PIXELS];  /* cleaned ridge image */
  uint8_t mask[E0C7F_PIXELS]; /* 1 where the finger covered the sensor */
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
