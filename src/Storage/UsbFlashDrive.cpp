
#if SUPPORT_USB_DRIVE
#include <Libraries/usbh_msc/usbh_msc.h>
#endif

#include "UsbFlashDrive.h"


uint64_t UsbFlashDrive::GetCapacity() const
{
	return 0;
}

uint32_t UsbFlashDrive::GetInterfaceSpeed() const
{
	return 0;
}


// #if SUPPORT_USB_DRIVE
// static UsbDriveInfo drives[NumUsbDrives];
// #endif


// GCodeResult MassStorage::Mount(size_t card, const StringRef& reply, bool reportSuccess) noexcept
// {
// #if SUPPORT_USB_DRIVE
// 	if (card >= NumSdCards)
// 	{
// 		if (usbDrivePresent())
// 		{
// 			UsbDriveInfo& inf = drives[card % NumSdCards];
// 			const char path[3] = { (char)('0' + card), ':', 0 };
// 			const FRESULT mounted = f_mount(&inf.fileSystem, path, 1);
// 		}
// 		return GCodeResult::error;
// 	}
// 	else
// #endif
// }
