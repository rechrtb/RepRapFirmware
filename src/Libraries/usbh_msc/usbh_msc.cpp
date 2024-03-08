#include <cstdint>

static bool mounted = false;

bool usbDriveMounted()
{
    return mounted;
}

void tuh_msc_mount_cb(uint8_t dev_addr)
{
    mounted = true;
}

void tuh_msc_umount_cb(uint8_t dev_addr)
{
    mounted = false;
}
