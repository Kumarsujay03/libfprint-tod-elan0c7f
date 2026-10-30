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
- **Enroll into a mosaic.** Each press is stitched onto the presses before
  it, building one larger image of the whole finger (the mosaic), with a
  per-pixel clarity map. Enrollment is open-ended: at least 10 presses, at
  most 30, and it ends once the finger is covered (about 3 presses' worth of
  area) and 4 presses in a row add almost nothing new. A press that doesn't
  overlap what is already captured is asked for again, closer in; one that
  is too partial or too blurry is asked for again too. The presses, which of
  them are stitched, and the mosaic are stored through fprintd's normal
  storage in `/var/lib/fprint`.
- **Match.** It compares a new press against the mosaic (and the stored
  presses) by normalised cross-correlation over rotations of ±24° and small
  shifts, weighted by how clearly each part of the finger was seen. A mosaic
  print accepts at 0.55; prints enrolled before mosaics existed still use
  their presses at 0.50. Matching runs on all CPU cores.
- **Early match.** While a press is still being captured, the first 3 frames
  are already matched. A clear match (0.62 or more, a stricter bar because
  fewer frames are averaged) ends the press there; otherwise the full 8-frame
  check runs as usual. A good press is decided in about 0.3 s.
- **Learning.** A confident match is blended into a learned copy of the
  finger's mosaic (and, for older prints, into up to 5 learned presses), so
  matching adapts to how the finger is really pressed. The driver also keeps
  a library of up to 100 good presses per finger. Presses that didn't fit the
  mosaic when they came are tried again whenever the mosaic grows, in a
  background thread. fprintd only saves prints at enrollment, so the driver
  keeps all of this itself, root-only, in `/var/lib/fprint/.elan0c7f-learned/`.
  Safeguards: the press must match the *enrolled* print (never only learned
  data) at 0.60 against a mosaic or 0.65 against a press, beat every other
  finger by 0.20, add something new, and be a clear, well-covered image. The
  enrolled print is never changed.
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
running when the screen locks. If you do, also install the resume hook:

```
sudo install -m 755 systemd/fprintd-resume /usr/lib/systemd/system-sleep/
```

Closing the lid while a fingerprint check is reading the sensor makes fprintd
abort it ("Cannot run while suspended") without releasing the device. An
always-running fprintd then keeps the sensor claimed, and every later check
fails until fprintd restarts. The hook restarts fprintd after each resume.

## Enroll

Use **Settings → System → Users → Fingerprint Login**, or the command line:

```
fprintd-enroll -f right-index-finger
fprintd-verify -f any
```

For good results:

- Press flat and hold still for about half a second, then lift fully.
- Move the finger only a little between presses (a few millimetres), working
  out from the centre to the tip, the edges and the lower part. Each press
  must overlap the last ones to be stitched in; a big jump is asked for again.
- Keep going until enrollment ends by itself (10 to 30 presses).
- Enroll each finger once. Re-enrolling a finger replaces its old print.
  Prints enrolled with an older version of this driver keep working, but
  re-enrolling turns them into mosaics, which match better.

## Security notes

Read these before you rely on the sensor:

- **No liveness detection.** The sensor returns a plain image. The driver
  cannot tell a real finger from a good fake.
- **Accuracy is not formally measured.** Against mosaics, genuine presses
  mostly scored 0.57–0.75 and other fingers stayed at or below 0.50, against
  the threshold of 0.55. In offline tests a thumb reached 0.50 against a
  20-press enrollment. That is one person's fingers, not a proper
  false-accept-rate study.
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
- `make mosaic-test` stitches recorded enrollment presses into a mosaic and
  scores right- and wrong-finger presses against it.
- `make SAVE_SAMPLES=1` keeps the raw frames of every enrollment press and a
  picture of the mosaic in `/var/lib/fprint/.elan0c7f-samples` (root only),
  so image processing can be improved later without enrolling again. These
  are images of your fingerprint: never share them, and don't use this build
  day to day.
- `make list-devices` builds a small tool that lists the readers libfprint
  sees.
- `make check` runs the unit test of the learned templates: storage, slot
  rotation, file permissions and every learning safeguard.
- `G_MESSAGES_DEBUG=all` on fprintd shows the driver's debug log.

The USB protocol is the one in libfprint's `elan` driver. Image processing and
matching are in `matcher.c`, and the device state machine is in `elan0c7f.c`.

## Licence

LGPL-2.1-or-later, the same as libfprint. See [LICENSE](LICENSE).
