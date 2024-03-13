#include <cstdint>

#include <tusb.h>
#include <class/msc/msc_host.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>

// #include <SEGGER_SYSVIEW_FreeRTOS.h>


static int mounted = false;
static scsi_inquiry_resp_t inquiry_resp;


bool usbDriveMounted()
{
    return mounted;
}

bool inquiry_complete_cb(uint8_t dev_addr, tuh_msc_complete_data_t const * cb_data)
{
    reprap.GetPlatform().MessageF(UsbMessage, "%.8s %.16s rev %.4s\r\n", inquiry_resp.vendor_id, inquiry_resp.product_id, inquiry_resp.product_rev);
    // traceEND();
	return true;
}

extern "C" void tuh_msc_mount_cb(uint8_t dev_addr)
{
    mounted = true;
    uint8_t const lun = 0;
    tuh_msc_inquiry(dev_addr, lun, &inquiry_resp, inquiry_complete_cb, 0);
}

extern "C" void tuh_msc_umount_cb(uint8_t dev_addr)
{
    mounted = false;
}
