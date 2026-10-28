# Security policy

This driver decides whether a finger unlocks a computer, so security
problems matter. Please **don't open a public issue** for one.

## Reporting

Use GitHub's private reporting: open the repository's **Security** tab and
choose **Report a vulnerability**
(<https://github.com/Kumarsujay03/libfprint-tod-elan0c7f/security/advisories/new>).
Only the maintainer can see the report. Please include the driver version
or commit, your distribution, and how to reproduce the problem.

You can expect a first reply within a week. Once a fix is released the
advisory is published, with credit to you unless you prefer otherwise.

## What counts

- A way to make the driver accept a finger that isn't enrolled, other than
  the known limits below.
- Crashes or memory errors triggered by the sensor's data or by stored
  templates.
- Templates or images being written or logged somewhere they shouldn't be.

## Known limits (not vulnerabilities)

These are described in the README and can't be fixed in the driver:

- The sensor has no liveness detection, so a good fake finger may be accepted.
- The false-accept rate hasn't been measured on a large set of fingers yet.
- fprintd stores templates in `/var/lib/fprint`, readable by root.
