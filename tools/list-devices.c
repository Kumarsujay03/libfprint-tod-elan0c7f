/* List fingerprint devices as the system libfprint sees them. */
#include <fprint.h>

int
main (void)
{
  g_autoptr(FpContext) ctx = fp_context_new ();
  GPtrArray *devices = fp_context_get_devices (ctx);

  for (guint i = 0; i < devices->len; i++)
    {
      FpDevice *dev = g_ptr_array_index (devices, i);
      g_print ("%s: %s (driver %s, %d enroll stages, %s)\n",
               fp_device_get_device_id (dev), fp_device_get_name (dev),
               fp_device_get_driver (dev), fp_device_get_nr_enroll_stages (dev),
               fp_device_get_scan_type (dev) == FP_SCAN_TYPE_PRESS ? "press" : "swipe");
    }
  if (devices->len == 0)
    g_print ("no devices\n");
  return 0;
}
