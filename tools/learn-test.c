/* Unit test for the learned templates: storage and the learning safeguards.
 * Includes the driver source to reach its static functions. */
#include "../elan0c7f.c"
#include <sys/stat.h>
#include <stdio.h>
#include <glib/gstdio.h>

static FpPrint *
make_print (guint8 seed)
{
  FpPrint *print = g_object_new (FP_TYPE_PRINT, "driver", "elan0c7f", "device-id", "test", "device-stored", FALSE, NULL);
  g_autoptr(GArray) t = g_array_new (FALSE, TRUE, sizeof (E0c7fTemplate));
  E0c7fTemplate one;
  memset (&one, seed, sizeof one);
  g_array_append_val (t, one);
  g_array_append_val (t, one);
  fpi_print_set_type (print, FPI_PRINT_RAW);
  g_object_set (print, "fpi-data", templates_to_variant (t), NULL);
  return print;
}

static MatchJob *
job_with (FpPrint *a, FpPrint *b, double *scores, guint8 *learned, int n, double cov, double clar)
{
  MatchJob *job = g_new0 (MatchJob, 1);
  job->action = FPI_DEVICE_ACTION_IDENTIFY;
  job->templates = g_array_new (FALSE, TRUE, sizeof (E0c7fTemplate));
  job->owners = g_ptr_array_new ();
  job->learned = g_byte_array_new ();
  job->scores = g_new0 (double, n);
  job->best_score = -1;
  for (int i = 0; i < n; i++)
    {
      E0c7fTemplate t = { 0 };
      g_array_append_val (job->templates, t);
      g_ptr_array_add (job->owners, i < n / 2 ? a : b);
      g_byte_array_append (job->learned, &learned[i], 1);
      job->scores[i] = scores[i];
      if (scores[i] > job->best_score)
        {
          job->best_score = scores[i];
          job->best_print = g_ptr_array_index (job->owners, i);
        }
    }
  job->probe_ok = TRUE;
  job->coverage = cov;
  job->clarity = clar;
  memset (&job->probe, 7, sizeof job->probe);
  return job;
}

static guint
count_for (FpPrint *p)
{
  guint next;
  g_autoptr(GArray) l = load_learned (p, &next);
  return l->len;
}

int
main (int argc, char **argv)
{
  g_autoptr(FpPrint) a = make_print (1), b = make_print (2);
  g_autofree gchar *path = NULL;
  guint next;
  struct stat st;
  int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf ("FAIL: %s\n", msg); fails++; } else printf ("ok: %s\n", msg); } while (0)

  g_setenv ("E0C7F_LEARN_DIR", argv[1], TRUE);
  path = learned_path (a);

  /* slots fill to LEARN_SLOTS, then the oldest is replaced */
  for (int i = 0; i < 7; i++)
    {
      E0c7fTemplate t;
      memset (&t, 10 + i, sizeof t);
      store_learned (a, &t);
    }
  {
    g_autoptr(GArray) l = load_learned (a, &next);
    CHECK (l->len == LEARN_SLOTS, "keeps at most 5 learned templates");
    CHECK (g_array_index (l, E0c7fTemplate, 0).img[0] == 15 &&
           g_array_index (l, E0c7fTemplate, 1).img[0] == 16 &&
           g_array_index (l, E0c7fTemplate, 2).img[0] == 12, "replaces the oldest first");
    CHECK (next == 2, "next slot wraps around");
  }
  CHECK (stat (path, &st) == 0 && (st.st_mode & 0777) == 0600, "file is private (0600)");
  CHECK (count_for (b) == 0, "other fingers are unaffected");

  /* templates loaded for matching: 2 enrolled + 5 learned, flagged */
  {
    g_autoptr(GArray) t = g_array_new (FALSE, TRUE, sizeof (E0c7fTemplate));
    g_autoptr(GPtrArray) o = g_ptr_array_new ();
    g_autoptr(GByteArray) l = g_byte_array_new ();
    add_print_templates (a, t, o, l);
    CHECK (t->len == 7 && l->data[0] == 0 && l->data[1] == 0 && l->data[2] == 1 && l->data[6] == 1,
           "matching uses enrolled + learned templates, flagged");
  }

  /* corrupt file is ignored, not trusted */
  g_file_set_contents (path, "junk", 4, NULL);
  CHECK (count_for (a) == 0, "a damaged file is ignored");
  g_unlink (path);

  /* decision safeguards: 4 templates of a (2 enrolled, 2 learned) + 4 of b */
  struct { const char *what; double s[8]; double cov, clar; gboolean learn; } cases[] = {
    { "clear match is learned",               { 0.70, 0.50, 0, 0, 0.30, 0.30, 0.30, 0.30 }, 0.85, 0.36, TRUE },
    { "borderline match (0.58) is not",       { 0.58, 0.50, 0, 0, 0.30, 0.30, 0.30, 0.30 }, 0.85, 0.36, FALSE },
    { "matching only a learned one is not",   { 0.40, 0.40, 0.80, 0, 0.30, 0.30, 0.30, 0.30 }, 0.85, 0.36, FALSE },
    { "too close to another finger is not",   { 0.70, 0.50, 0, 0, 0.55, 0.30, 0.30, 0.30 }, 0.85, 0.36, FALSE },
    { "near-duplicate (0.93) is not",         { 0.93, 0.50, 0, 0, 0.30, 0.30, 0.30, 0.30 }, 0.85, 0.36, FALSE },
    { "partial press is not",                 { 0.70, 0.50, 0, 0, 0.30, 0.30, 0.30, 0.30 }, 0.60, 0.36, FALSE },
    { "blurry press is not",                  { 0.70, 0.50, 0, 0, 0.30, 0.30, 0.30, 0.30 }, 0.85, 0.20, FALSE },
  };
  guint8 learned[8] = { 0, 0, 1, 1, 0, 0, 0, 0 };
  for (guint c = 0; c < G_N_ELEMENTS (cases); c++)
    {
      MatchJob *job = job_with (a, b, cases[c].s, learned, 8, cases[c].cov, cases[c].clar);
      guint before = count_for (a);
      maybe_learn (job);
      CHECK ((count_for (a) > before) == cases[c].learn, cases[c].what);
      match_job_free (job);
    }
  printf ("%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
  return fails != 0;
}
