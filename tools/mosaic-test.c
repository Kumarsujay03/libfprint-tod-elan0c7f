/*
 * Offline check of the mosaic on recorded raw frame dumps.
 * Usage: mosaic-test <enroll dirs...> -- <genuine probe dirs...> -- <wrong-finger probe dirs...>
 * Stitches the enroll presses as enrollment does, then scores every probe
 * against the mosaic and, for comparison, against the separate presses.
 */

#include "matcher.h"

#include <glob.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>



#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#define MAX_ISLANDS 32

static int
load_press (const char *dir, E0c7fTemplate *out)
{
  char pattern[4096];
  glob_t g;
  uint16_t *frames;
  int n = 0, ret;
  int max = getenv ("E0C7F_MAX_FRAMES") ? atoi (getenv ("E0C7F_MAX_FRAMES")) : 8;

  snprintf (pattern, sizeof pattern, "%s/raw-*.bin", dir);
  if (glob (pattern, 0, NULL, &g) != 0)
    return -1;
  frames = malloc (g.gl_pathc * E0C7F_PIXELS * sizeof (uint16_t));
  for (size_t i = 0; i < g.gl_pathc && n < max; i++)
    {
      FILE *f = fopen (g.gl_pathv[i], "rb");
      int c = 0;
      if (f && fread (frames + n * E0C7F_PIXELS, 2, E0C7F_PIXELS, f) == E0C7F_PIXELS)
        {
          for (int k = 0; k < E0C7F_PIXELS; k++)
            c += frames[n * E0C7F_PIXELS + k] > 0;
          n += c > 0.3 * E0C7F_PIXELS; /* skip touch-down frames */
        }
      if (f)
        fclose (f);
    }
  globfree (&g);
  ret = e0c7f_template_from_frames (frames, n, out);
  free (frames);
  return ret;
}

static double
now (void)
{
  struct timespec t;
  clock_gettime (CLOCK_MONOTONIC, &t);
  return t.tv_sec + t.tv_nsec / 1e9;
}

int
main (int argc, char **argv)
{

  E0c7fTemplate *enr = calloc (argc, sizeof *enr);
  E0c7fMosaic *isl[MAX_ISLANDS];
  int nenr = 0, nisl = 0, a = 1;
  double t_mosaic = 0, t_sep = 0;
  int nmatch = 0;

  for (; a < argc && strcmp (argv[a], "--"); a++)
    if (load_press (argv[a], &enr[nenr]) == 0)
      nenr++;
    else
      fprintf (stderr, "skip %s\n", argv[a]);

  /* stitch like enrollment: onto the mosaic if it fits, else keep the press
   * aside and retry it whenever the mosaic has grown */
  int *loose = calloc (nenr, sizeof (int)), nloose = 0;
  isl[0] = e0c7f_mosaic_new (&enr[0]);
  nisl = 1;
  for (int i = 1; i < nenr; i++)
    {
      E0c7fPlace p;
      int added = 0;
      loose[nloose++] = i;
      for (int grew = 1; grew;)
        {
          grew = 0;
          for (int k = 0; k < nloose; k++)
            if (e0c7f_mosaic_match (isl[0], &enr[loose[k]], E0C7F_STITCH_OVERLAP, &p) >= E0C7F_STITCH_MIN)
              {
                added += e0c7f_mosaic_add (isl[0], &enr[loose[k]], &p);
                loose[k--] = loose[--nloose];
                grew = 1;
              }
        }
      printf ("press %2d: new area %5d px (%3.0f%% of a press), mosaic %6d px, %d presses not stitched\n",
              i + 1, added, 100.0 * added / E0C7F_PIXELS, e0c7f_mosaic_area (isl[0]), nloose);
    }
  for (int k = 0; k < nisl; k++)
    printf ("island %d: %dx%d, %d px known\n", k, isl[k]->w, isl[k]->h, e0c7f_mosaic_area (isl[k]));

  for (int kind = 0; kind < 2 && a < argc; kind++)
    {
      printf ("%s probes (mosaic / separate presses):\n", kind ? "wrong-finger" : "genuine");
      for (a++; a < argc && strcmp (argv[a], "--"); a++)
        {
          E0c7fTemplate p;
          double sm = -1, ss = -1, t0;
          if (load_press (argv[a], &p) != 0)
            continue;
          t0 = now ();
          for (int k = 0; k < nisl; k++)
            sm = MAX (sm, e0c7f_mosaic_match (isl[k], &p, E0C7F_MATCH_OVERLAP, NULL));
          for (int k = 0; k < nloose; k++)
            sm = MAX (sm, e0c7f_match (&enr[loose[k]], &p));
          t_mosaic += now () - t0;
          t0 = now ();
          for (int i = 0; i < nenr; i++)
            ss = MAX (ss, e0c7f_match (&enr[i], &p));
          t_sep += now () - t0;
          nmatch++;
          printf ("  %.3f  %.3f  %s\n", sm, ss, argv[a]);
        }
    }
  if (nmatch)
    printf ("time per probe: mosaic %.0f ms, separate %.0f ms (one thread)\n",
            1000 * t_mosaic / nmatch, 1000 * t_sep / nmatch);
  return 0;
}
