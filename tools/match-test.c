/*
 * Offline check of the matcher on recorded raw frame dumps.
 * Usage: match-test <press_dir> [<press_dir> ...]
 * Prints the full score matrix (row = probe, column = enrolled) as CSV.
 */

#include "matcher.h"

#include <glob.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>


static int
load_press (const char *dir, E0c7fTemplate *out)
{
  char pattern[4096];
  glob_t g;
  uint16_t *frames;
  int n = 0;

  snprintf (pattern, sizeof pattern, "%s/raw-*.bin", dir);
  if (glob (pattern, 0, NULL, &g) != 0)
    return -1;
  frames = malloc (g.gl_pathc * E0C7F_PIXELS * sizeof (uint16_t));
  for (size_t i = 0; i < g.gl_pathc; i++)
    {
      FILE *f = fopen (g.gl_pathv[i], "rb");
      if (f && fread (frames + n * E0C7F_PIXELS, 2, E0C7F_PIXELS, f) == E0C7F_PIXELS)
        n++;
      if (f)
        fclose (f);
    }
  globfree (&g);

  /* emulate a shorter capture: keep only the first N frames with a finger */
  const char *max_env = getenv ("E0C7F_MAX_FRAMES");
  if (max_env)
    {
      int max = atoi (max_env), kept = 0;
      for (int f = 0; f < n && kept < max; f++)
        {
          int c = 0;
          for (int i = 0; i < E0C7F_PIXELS; i++)
            c += frames[f * E0C7F_PIXELS + i] > 0;
          if (c > 0.3 * E0C7F_PIXELS)
            memmove (frames + kept++ * E0C7F_PIXELS, frames + f * E0C7F_PIXELS,
                     E0C7F_PIXELS * sizeof (uint16_t));
        }
      n = kept;
    }

  int ret = e0c7f_template_from_frames (frames, n, out);
  free (frames);
  return ret;
}

int
main (int argc, char **argv)
{
  int n = argc - 1;
  E0c7fTemplate *t = calloc (n, sizeof (E0c7fTemplate));
  struct timespec t0, t1;
  double secs = 0;
  int pairs = 0;

  for (int i = 0; i < n; i++)
    if (load_press (argv[i + 1], &t[i]) != 0)
      fprintf (stderr, "no usable frames in %s\n", argv[i + 1]);

  /* E0C7F_QUALITY=1: print "name coverage clarity" per press and stop */
  if (getenv ("E0C7F_QUALITY"))
    {
      for (int i = 0; i < n; i++)
        {
          double cov, clar;
          e0c7f_template_quality (&t[i], &cov, &clar);
          printf ("%s %.3f %.3f\n", argv[i + 1], cov, clar);
        }
      return 0;
    }

  for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < n; j++)
        {
          double s = 0;
          if (i != j)
            {
              clock_gettime (CLOCK_MONOTONIC, &t0);
              s = e0c7f_match (&t[j], &t[i]);
              clock_gettime (CLOCK_MONOTONIC, &t1);
              secs += (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
              pairs++;
            }
          printf ("%s%.4f", j ? "," : "", i == j ? NAN : s);
        }
      printf ("\n");
    }
  fprintf (stderr, "%.1f ms per match\n", 1000 * secs / pairs);
  return 0;
}
