# Build the ELAN 04f3:0c7f libfprint TOD plugin.
#
#   make                 build the plugin
#   sudo make install    install it where libfprint looks for TOD drivers
#   sudo make uninstall  remove it again
#
# Needs: libfprint-2-tod-dev libfprint-2-dev libglib2.0-dev libgusb-dev
# Options:
#   LOG_SCORES=1   log every match score to the system log (for measuring
#                  accuracy; leave off for normal use)
#   TOD_ROOT=dir   use unpacked -dev packages under dir instead of installed ones

PLUGIN  := libfprint-tod-elan0c7f.so
PKGS    := glib-2.0 gobject-2.0 gio-2.0 gmodule-2.0 gusb

ifdef TOD_ROOT
  LIBDIR     := /usr/lib/x86_64-linux-gnu
  FP_CFLAGS  := -I$(TOD_ROOT)/usr/include/libfprint-2 -I$(TOD_ROOT)/usr/include/libfprint-2/tod-1
  FP_LIBS    := $(LIBDIR)/libfprint-2-tod.so.1 $(LIBDIR)/libfprint-2.so.2
  TODDIR     := $(LIBDIR)/libfprint-2/tod-1
else
  FP_CFLAGS  := $(shell pkg-config --cflags libfprint-2-tod-1 libfprint-2 2>/dev/null)
  FP_LIBS    := $(shell pkg-config --libs libfprint-2-tod-1 libfprint-2 2>/dev/null)
  TODDIR     := $(shell pkg-config --variable=tod_driversdir libfprint-2-tod-1 2>/dev/null)
endif

CFLAGS  ?= -O2 -g
CFLAGS  += -fPIC -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers \
           $(FP_CFLAGS) $(shell pkg-config --cflags $(PKGS))
ifeq ($(LOG_SCORES),1)
  CFLAGS += -DE0C7F_LOG_SCORES
endif
LDLIBS  := $(FP_LIBS) $(shell pkg-config --libs $(PKGS)) -lm

all: $(PLUGIN)

$(PLUGIN): elan0c7f.c matcher.c matcher.h
	$(CC) $(CFLAGS) -shared -Wl,--no-undefined -o $@ elan0c7f.c matcher.c $(LDLIBS)

# offline matcher check on recorded raw frames (see tools/)
match-test: tools/match-test.c matcher.c matcher.h
	$(CC) -O2 -Wall -Wextra -I. -o $@ tools/match-test.c matcher.c -lm

# lists fingerprint devices as the system libfprint sees them
list-devices: tools/list-devices.c
	$(CC) -O2 -Wall -o $@ tools/list-devices.c $(FP_CFLAGS) $(shell pkg-config --cflags --libs glib-2.0 gobject-2.0 gio-2.0) $(filter %libfprint-2.so.2 -lfprint-2,$(FP_LIBS))

install: $(PLUGIN)
	@test -n "$(TODDIR)" || { echo "libfprint TOD driver directory not found"; exit 1; }
	install -D -m 644 $(PLUGIN) $(DESTDIR)$(TODDIR)/$(PLUGIN)
	@echo "Installed. Restart fprintd: sudo systemctl restart fprintd"

uninstall:
	rm -f $(DESTDIR)$(TODDIR)/$(PLUGIN)

clean:
	rm -f $(PLUGIN) match-test list-devices

.PHONY: all install uninstall clean
