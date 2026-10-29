# libfprint-tod-elan0c7f

[![Build](https://github.com/Kumarsujay03/libfprint-tod-elan0c7f/actions/workflows/build.yml/badge.svg)](https://github.com/Kumarsujay03/libfprint-tod-elan0c7f/actions/workflows/build.yml)

A Linux driver for the **ELAN 04f3:0c7f** fingerprint sensor, as fitted to the
Acer Swift 3 SF314-512. Upstream libfprint does not support this sensor. This
driver is a [libfprint TOD](https://gitlab.freedesktop.org/3v1n0/libfprint/-/tree/tod)
plugin, so it works with the stock `fprintd`, GNOME Settings, the GDM login
screen and the lock screen, like any other fingerprint reader.

Check that you have this sensor:

```
$ lsusb | grep 04f3:0c7f
Bus 003 Device 004: ID 04f3:0c7f Elan Microelectronics Corp. ELAN:Fingerprint
```

## How it works

The sensor speaks the same USB protocol as the ELAN image sensors that the
upstream `elan` driver already handles. It is a small **press** sensor
(52 × 150 pixels), though, and that patch of finger is too small for the
minutiae matching libfprint normally uses. This driver does the following:

- **Capture.** It averages several frames of each press, removes the sensor's
  row/column offsets and shading, and normalises the ridge contrast.
- **Enroll.** It records 15 presses, each of a different part of the finger.
  The cleaned images are stored as the template, through fprintd's normal
  storage in `/var/lib/fprint`. A press that is too partial or too blurry is
  asked for again.
- **Match.** It compares a new press against each stored press by normalised
  cross-correlation over rotations of ±24° and small shifts, and accepts the
  best score at or above 0.50. Matching runs on all CPU cores.
- **Early match.** While a press is still being captured, the first 3 frames
  are already matched. A clear match (0.62 or more, a stricter bar because
  fewer frames are averaged) ends the press there; otherwise the full 8-frame
  check runs as usual. A good press is decided in about 0.3 s.
- **Learning.** A confident match adds that press to up to 5 extra "learned"
  templates for the finger, replacing the oldest, so matching adapts to how
  the finger is really pressed. fprintd only saves prints at enrollment, so
  the driver keeps these itself, root-only, in
  `/var/lib/fprint/.elan0c7f-learned/`. Safeguards: the press must match an
  *enrolled* template (never only a learned one) at 0.65 or more, beat every
  other finger by 0.20, not be a near-copy of an existing template, and be a
  clear, well-covered image. Enrolled templates are never changed.
- **Always ready.** `make install` also adds a udev rule that keeps the sensor
  out of USB power saving, so the first touch is read at once instead of
  after a wake-up.

## Install

Tested on Ubuntu 26.04 (`libfprint-2-tod1` 1.95). Other distributions with
libfprint-TOD should work too.

```
sudo apt install build-essential pkg-config fprintd libpam-fprintd \
    libfprint-2-tod1 libfprint-2-tod-dev libfprint-2-dev libgusb-dev
make
sudo make install
sudo udevadm trigger
sudo systemctl restart fprintd
```

The plugin goes into libfprint's TOD driver directory
(`/usr/lib/x86_64-linux-gnu/libfprint-2/tod-1/`). Check that it loaded:

```
fprintd-list $USER
```

To remove it: `sudo make uninstall && sudo systemctl restart fprintd`. To
forget what the driver has learned, delete `/var/lib/fprint/.elan0c7f-learned`.

Optional, for a lock screen that reacts at once: start fprintd with
`--no-timeout` (a systemd drop-in for `fprintd.service`), so it is already
running when the screen locks.

## Enroll

Use **Settings → System → Users → Fingerprint Login**, or the command line:

```
fprintd-enroll -f right-index-finger
fprintd-verify -f any
```

For good results:

- Press flat and hold still for about half a second, then lift fully.
- Move the finger between presses: the tip, the left and right edges, the
  centre and the lower part, so the template covers the whole finger.
- Enroll each finger once. Re-enrolling a finger replaces its old template.

## Security notes

Read these before you rely on the sensor:

- **No liveness detection.** The sensor returns a plain image. The driver
  cannot tell a real finger from a good fake.
- **Accuracy is not formally measured.** In testing, genuine presses scored
  0.58–0.73 and other fingers stayed at or below about 0.43, against the
  threshold of 0.50, with no false accepts in about 40 wrong-finger tries.
  That is one person's fingers, not a proper false-accept-rate study.
- **Templates are images.** The stored and learned templates are cleaned
  ridge images of your finger, readable by root only, as with any image-based
  libfprint driver. Nothing leaves the machine.
- **Learning changes what is accepted over time.** It only learns from clear
  matches against enrolled templates, but if you are worried about a false
  accept being learned, delete the learned folder; enrolled templates are
  never affected.

Because of this, we suggest using the fingerprint for the **login and lock
screen only**, and keeping a password for `sudo`, polkit and keyring unlock.
On Ubuntu, `sudo pam-auth-update` controls where fingerprint login is used.

Please report security problems privately, as described in
[SECURITY.md](SECURITY.md), not in public issues.

## Development

- `make LOG_SCORES=1` logs every match score to the system journal
  (`journalctl -u fprintd`). Use it to measure accuracy on your own fingers,
  and don't leave it on for daily use.
- `make match-test` builds an offline tool that prints the score matrix for
  presses recorded as raw frame dumps: `raw-*.bin` files of 52 × 150
  little-endian 16-bit pixels, one directory per press.
- `make list-devices` builds a small tool that lists the readers libfprint
  sees.
- `make check` runs the unit test of the learned templates: storage, slot
  rotation, file permissions and every learning safeguard.
- `G_MESSAGES_DEBUG=all` on fprintd shows the driver's debug log.

The USB protocol is the one in libfprint's `elan` driver. Image processing and
matching are in `matcher.c`, and the device state machine is in `elan0c7f.c`.

## Licence

LGPL-2.1-or-later, the same as libfprint. See [LICENSE](LICENSE).
