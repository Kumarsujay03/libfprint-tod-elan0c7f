/* Unit test for the learned templates: storage and the learning safeguards.
 * Includes the driver source to reach its static functions. */
#include "../elan0c7f.c"
#include <math.h>
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
empty_job (void)
{
  MatchJob *job = g_new0 (MatchJob, 1);
  job->action = FPI_DEVICE_ACTION_IDENTIFY;
  job->templates = g_array_new (FALSE, TRUE, sizeof (E0c7fTemplate));
  job->owners = g_ptr_array_new ();
  job->learned = g_byte_array_new ();
  job->mosaics = g_ptr_array_new_with_free_func ((GDestroyNotify) e0c7f_mosaic_free);
  job->mosaic_owners = g_ptr_array_new ();
  job->mosaic_learned = g_byte_array_new ();
  job->best_score = -1;
  return job;
}

static MatchJob *
job_with (FpPrint *a, FpPrint *b, double *scores, guint8 *learned, int n, double cov, double clar)
{
  MatchJob *job = empty_job ();
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

#define CHECK(c, msg) do { if (!(c)) { printf ("FAIL: %s\n", msg); fails++; } else printf ("ok: %s\n", msg); } while (0)

/* ---- mosaics -------------------------------------------------------- */

/* a press-sized window at (x0, y0) of a synthetic finger: ridges whose
 * direction and spacing wander, so no two places look alike */
static void
synthetic_press (int finger, int x0, int y0, E0c7fTemplate *t)
{
  /* per finger: ridge direction, and the scale and twist of the wander */
  static const double f[2][5] = { { 0.55, 0.30, 21, 29, 17 }, { -0.25, 0.58, 13, 37, 11 } };
  const double *c = f[finger];
  for (int y = 0; y < E0C7F_H; y++)
    for (int x = 0; x < E0C7F_W; x++)
      {
        double X = x + x0, Y = y + y0;
        double phase = X * c[0] + Y * c[1] + 7 * sin (X / c[2]) + 6 * cos (Y / c[3]) +
                       4 * sin ((X + 2 * Y) / c[4]) + 3 * sin (X * Y / 2500.0);
        t->img[y * E0C7F_W + x] = (uint8_t) (127.5 + 120 * sin (phase));
        t->mask[y * E0C7F_W + x] = 1;
      }
}

static MatchJob *
enroll_job (void)
{
  MatchJob *job = empty_job ();
  job->action = FPI_DEVICE_ACTION_ENROLL;
  job->enroll_templates = g_array_new (FALSE, FALSE, sizeof (E0c7fTemplate));
  job->enroll_stitched = g_byte_array_new ();
  return job;
}

/* stitch one more press, as the thread does during enrollment */
static void
enroll_press (MatchJob *job, int finger, int x0, int y0)
{
  job->enroll_added = 0;
  job->enroll_too_far = FALSE;
  synthetic_press (finger, x0, y0, &job->probe);
  stitch_press (job);
}

static FpPrint *
mosaic_print (MatchJob *enrolled)
{
  FpPrint *print = g_object_new (FP_TYPE_PRINT, "driver", "elan0c7f", "device-id", "test",
                                 "device-stored", FALSE, NULL);
  fpi_print_set_type (print, FPI_PRINT_RAW);
  g_object_set (print, "fpi-data",
                mosaic_print_to_variant (enrolled->enroll_templates, enrolled->enroll_stitched,
                                         enrolled->enroll_mosaic), NULL);
  return print;
}

/* learn a synthetic press of finger 0 at (x0, y0) for pa, as if it matched
 * pa's enrolled mosaic clearly (0.72) and pb's at 0.30 */
static void
learn_press (FpPrint *pa, FpPrint *pb, int x0, int y0)
{
  MatchJob *job = empty_job ();
  add_print_templates (pa, job);
  add_print_templates (pb, job);
  synthetic_press (0, x0, y0, &job->probe);
  job->probe_ok = TRUE;
  job->coverage = 0.9;
  job->clarity = 0.4;
  job->scores = g_new0 (double, job->templates->len);
  for (guint i = 0; i < job->templates->len; i++)
    job->scores[i] = -1;
  job->mosaic_scores = g_new0 (double, job->mosaics->len);
  job->mosaic_places = g_new0 (E0c7fPlace, job->mosaics->len);
  for (guint i = 0; i < job->mosaics->len; i++)
    job->mosaic_scores[i] = job->mosaic_owners->pdata[i] == pa ? 0.72 : 0.30;
  job->best_score = 0.72;
  job->best_print = pa;
  maybe_learn (job);
  match_job_free (job);
}

static void
library_counts (FpPrint *print, guint *total, guint *waiting)
{
  g_autofree gchar *path = learned_file (print, "presses");
  guint next;
  g_autoptr(GArray) lib = load_library (path, &next);
  *total = lib->len;
  *waiting = library_waiting (lib);
}

static int
mosaic_checks (const char *dir)
{
  int fails = 0;
  MatchJob *e = enroll_job (), *e2 = enroll_job ();
  int area;

  /* enrollment: overlapping presses grow one mosaic */
  enroll_press (e, 0, 60, 100);
  area = e0c7f_mosaic_area (e->enroll_mosaic);
  enroll_press (e, 0, 60, 160);
  CHECK (e->enroll_fits && e->enroll_added > 0.2 * E0C7F_PIXELS,
         "a press reaching further is stitched and adds area");
  enroll_press (e, 0, 90, 130);
  CHECK (e->enroll_fits && e0c7f_mosaic_area (e->enroll_mosaic) > area * 1.5,
         "the mosaic grows past one press");
  area = e0c7f_mosaic_area (e->enroll_mosaic);
  enroll_press (e, 0, 62, 130);
  CHECK (e->enroll_fits && e->enroll_added < ENROLL_REPEAT_AREA * E0C7F_PIXELS,
         "a press of a known part counts as a repeat");

  /* a far part does not fit: it is asked for again, closer in */
  enroll_press (e, 0, 60, 330);
  CHECK (e->enroll_too_far && e->enroll_templates->len == 4, "a far press is asked for again");
  /* after repeated misses it is kept aside, and stitched once a press
   * bridges to it */
  e->enroll_misses = ENROLL_MAX_MISSES;
  enroll_press (e, 0, 60, 330);
  e->enroll_misses = 0;
  CHECK (!e->enroll_too_far && !e->enroll_fits && e->enroll_stitched->data[4] == 0,
         "after repeated misses a far press is kept aside");
  enroll_press (e, 0, 60, 250);
  CHECK (e->enroll_fits && e->enroll_stitched->data[4] == 1,
         "the kept press is stitched once the mosaic reaches it");

  /* matching against the mosaic */
  {
    E0c7fTemplate between, other;
    E0c7fPlace at;
    synthetic_press (0, 75, 200, &between);
    synthetic_press (1, 75, 200, &other);
    CHECK (e0c7f_mosaic_match (e->enroll_mosaic, &between, E0C7F_MATCH_OVERLAP, &at) > 0.9,
           "a press between enrolled presses matches the mosaic");
    CHECK (e0c7f_mosaic_match (e->enroll_mosaic, &other, E0C7F_MATCH_OVERLAP, NULL) < 0.4,
           "another finger does not");
  }

  /* the print keeps every press, the stitched flags and the mosaic; matching
   * loads the mosaic plus only the presses that are not stitched */
  e->enroll_misses = ENROLL_MAX_MISSES;
  enroll_press (e, 1, 60, 100); /* another finger: never fits */
  enroll_press (e2, 1, 60, 100);
  enroll_press (e2, 1, 60, 150);
  g_setenv ("E0C7F_LEARN_DIR", dir, TRUE);
  {
    g_autoptr(FpPrint) pa = mosaic_print (e), pb = mosaic_print (e2);
    g_autofree gchar *mpath = learned_file (pa, "mosaic");
    MatchJob *job = empty_job ();
    struct stat st;

    CHECK (add_print_templates (pa, job) && job->mosaics->len == 1 && job->templates->len == 1 &&
           ((E0c7fMosaic *) job->mosaics->pdata[0])->w == e->enroll_mosaic->w,
           "a mosaic print loads its mosaic and the unstitched press");
    match_job_free (job);

    /* learning: a clear match is blended into a learned copy */
    job = empty_job ();
    add_print_templates (pa, job);
    add_print_templates (pb, job);
    synthetic_press (0, 70, 180, &job->probe);
    job->probe_ok = TRUE;
    job->coverage = 0.9;
    job->clarity = 0.4;
    job->scores = g_new0 (double, job->templates->len);
    job->mosaic_scores = g_new0 (double, job->mosaics->len);
    job->mosaic_places = g_new0 (E0c7fPlace, job->mosaics->len);
    for (guint i = 0; i < job->mosaics->len; i++)
      job->mosaic_scores[i] = e0c7f_mosaic_match (job->mosaics->pdata[i], &job->probe,
                                                  E0C7F_MATCH_OVERLAP, &job->mosaic_places[i]);
    job->best_score = job->mosaic_scores[0];
    job->best_print = pa;
    maybe_learn (job);
    CHECK (stat (mpath, &st) == 0 && (st.st_mode & 0777) == 0600,
           "a clear match is learned into a private mosaic file");
    match_job_free (job);

    job = empty_job ();
    add_print_templates (pa, job);
    CHECK (job->mosaics->len == 2 && job->mosaic_learned->data[0] == 0 &&
           job->mosaic_learned->data[1] == 1,
           "matching then uses the enrolled and the learned mosaic, flagged");
    match_job_free (job);

    /* learning safeguards for mosaics: [enrolled a, learned a, b] */
    struct { const char *what; double s[3]; gboolean learn; } cases[] = {
      { "mosaic: clear match is learned",          { 0.72, 0.74, 0.30 }, TRUE },
      { "mosaic: matching only the learned one is not", { 0.55, 0.80, 0.30 }, FALSE },
      { "mosaic: too close to another finger is not",   { 0.72, 0.74, 0.60 }, FALSE },
    };
    for (guint c = 0; c < G_N_ELEMENTS (cases); c++)
      {
        guint before = 0, after = 0;
        e0c7f_mosaic_free (load_learned_mosaic (pa, &before));
        job = empty_job ();
        add_print_templates (pa, job);
        add_print_templates (pb, job);
        synthetic_press (0, 70, 180, &job->probe);
        job->probe_ok = TRUE;
        job->coverage = 0.9;
        job->clarity = 0.4;
        job->scores = g_new0 (double, job->templates->len);
        for (guint i = 0; i < job->templates->len; i++)
          job->scores[i] = -1;
        job->mosaic_scores = g_new0 (double, job->mosaics->len);
        job->mosaic_places = g_new0 (E0c7fPlace, job->mosaics->len);
        for (guint i = 0; i < job->mosaics->len; i++)
          {
            e0c7f_mosaic_match (job->mosaics->pdata[i], &job->probe, E0C7F_MATCH_OVERLAP,
                                &job->mosaic_places[i]);
            job->mosaic_scores[i] = cases[c].s[MIN (i, 2)];
          }
        job->best_score = MAX (cases[c].s[0], cases[c].s[1]);
        job->best_print = pa;
        maybe_learn (job);
        e0c7f_mosaic_free (load_learned_mosaic (pa, &after));
        CHECK ((after > before) == cases[c].learn, cases[c].what);
        match_job_free (job);
      }

    g_file_set_contents (mpath, "junk", 4, NULL);
    job = empty_job ();
    add_print_templates (pa, job);
    CHECK (job->mosaics->len == 1, "a damaged learned mosaic is ignored");
    match_job_free (job);

    /* the library: a clear press beyond the mosaic waits; once a press
     * grows the mosaic towards it, it is fitted in */
    {
      g_autofree gchar *lpath = learned_file (pa, "presses");
      guint total, waiting, before = 0, after = 0;

      g_unlink (mpath);
      g_unlink (lpath);
      learn_press (pa, pb, 60, 450);
      library_counts (pa, &total, &waiting);
      CHECK (stat (lpath, &st) == 0 && (st.st_mode & 0777) == 0600,
             "good presses are kept in a private library");
      CHECK (total == 2 && waiting == 2 && !g_file_test (mpath, G_FILE_TEST_EXISTS),
             "a press beyond the mosaic waits, with the enrolled press that never stitched");
      learn_press (pa, pb, 60, 370);
      e0c7f_mosaic_free (load_learned_mosaic (pa, &after));
      library_counts (pa, &total, &waiting);
      CHECK (after == 1 && total == 3 && waiting == 1,
             "a press that grows the mosaic fits the waiting one in (another finger's never fits)");
      (void) before;

      g_file_set_contents (lpath, "junk", 4, NULL);
      library_counts (pa, &total, &waiting);
      CHECK (total == 0, "a damaged library is ignored");
    }
  }
  match_job_free (e);
  match_job_free (e2);
  return fails;
}

int
main (int argc, char **argv)
{
  g_autoptr(FpPrint) a = make_print (1), b = make_print (2);
  g_autofree gchar *path = NULL;
  guint next;
  struct stat st;
  int fails = 0;

  g_setenv ("E0C7F_LEARN_DIR", argv[1], TRUE);
  learn_in_background = FALSE; /* check the files right after learning */
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
    MatchJob *job = empty_job ();
    guint8 *l;
    add_print_templates (a, job);
    l = job->learned->data;
    CHECK (job->templates->len == 7 && l[0] == 0 && l[1] == 0 && l[2] == 1 && l[6] == 1 &&
           job->mosaics->len == 0,
           "matching uses enrolled + learned templates, flagged");
    match_job_free (job);
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
  fails += mosaic_checks (argv[1]);
  printf ("%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
  return fails != 0;
}
