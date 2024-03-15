
#if SUPPORT_USB_DRIVE
#include <Libraries/usbh_msc/usbh_msc.h>
#endif

#if SUPPORT_USB_DRIVE
enum class UsbDriveState : uint8_t
{
	none = 0,
	inserted,
	mounted
};
#endif

#if SUPPORT_USB_DRIVE
struct UsbDriveInfo
{
	FATFS fileSystem;
	Mutex volMutex;
	uint32_t timer;
	uint16_t seq;
	UsbDriveState driveState;
};
#endif

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