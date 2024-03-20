
#include <cstdint>

#include <Platform/Platform.h>
#include <Platform/RepRap.h>

#include <tusb.h>
#include <class/msc/msc_host.h>

#include "UsbFlashDrive.h"

// #include <SEGGER_SYSVIEW_FreeRTOS.h>

uint64_t UsbFlashDrive::GetCapacity() const
{
  // Get capacity of device
//   uint32_t const block_count = tuh_msc_get_block_count(address, cbw->lun);
//   uint32_t const block_size = tuh_msc_get_block_size(address, cbw->lun);
//   return block_count / ((1024*1024)/block_size);
	return 0;
}

uint32_t UsbFlashDrive::GetInterfaceSpeed() const
{
	// tusb_speed_t speed = tuh_speed_get(address);
	// speed == TUSB_SPEED_HIGH ? 480000000 : 12000000;
	return 0;
}

void UsbFlashDrive::Spin() noexcept
{
	// Do nothing here, since it is managed by tinyUSB stack
}

GCodeResult UsbFlashDrive::Mount(size_t num, const StringRef &reply, bool reportSuccess) noexcept
{
	if (!IsPresent())
	{
		reply.copy("No USB flash drive present");
		return GCodeResult::error;
	}

	if (IsMounted())
	{
		reply.copy("USB already mounted"); // TODO: properly handle already mounted
		return GCodeResult::error;
	}

	// Mount the file systems
	const FRESULT mounted = f_mount(&fileSystem, path, 1);
	if (mounted == FR_NO_FILESYSTEM)
	{
		reply.printf("Cannot mount SD card %u: no FAT filesystem found on card (EXFAT is not supported)", volume);
		return GCodeResult::error;
	}
	if (mounted != FR_OK)
	{
		reply.printf("Cannot mount SD card %u: code %d", volume, mounted);
		return GCodeResult::error;
	}

	isMounted = true;
	return GCodeResult::ok;
}

unsigned int UsbFlashDrive::Unmount() noexcept
{
	MutexLocker lock1(MassStorage::GetFsMutex());
	MutexLocker lock2(volMutex);
	const unsigned int invalidated = MassStorage::InvalidateFiles(&fileSystem);
	f_mount(nullptr, path, 0);
	memset(&fileSystem, 0, sizeof(fileSystem));
	isMounted = false;
	reprap.VolumesUpdated();
	++seqNum;
	return invalidated;
}

void UsbFlashDrive::Init() noexcept
{
	StorageDevice::Init();
	address = 0;
	usbDrives[volume % NumSdCards] = this;
}

static volatile bool _disk_busy[NumUsbDrives];

static bool disk_io_complete(uint8_t dev_addr, tuh_msc_complete_data_t const *cb_data)
{
	(void)dev_addr;
	(void)cb_data;
	_disk_busy[dev_addr - 1] = false;
	return true;
}

static void wait_for_disk_io(BYTE pdrv)
{
	while (_disk_busy[pdrv])
	{
		delay(SdCardRetryDelay);
	}
}

DRESULT UsbFlashDrive::DiskInitialize()
{
	return RES_OK; // nothing to do
}

DRESULT UsbFlashDrive::DiskStatus()
{
	uint8_t dev_addr = (volume % NumSdCards) + 1;
	return static_cast<DRESULT>(tuh_msc_mounted(dev_addr) ? 0 : STA_NODISK);
}

DRESULT UsbFlashDrive::DiskRead(BYTE *buff, LBA_t sector, UINT count)
{
	uint8_t const dev_addr = (volume % NumSdCards) + 1;
	uint8_t const lun = 0;

	_disk_busy[volume % NumSdCards] = true;
	tuh_msc_read10(dev_addr, lun, buff, sector, (uint16_t)count, disk_io_complete, 0);
	wait_for_disk_io(volume % NumSdCards);

	return RES_OK;
}

DRESULT UsbFlashDrive::DiskWrite(BYTE const *buff, LBA_t sector, UINT count)
{
	uint8_t const dev_addr = (volume % NumSdCards) + 1;
	uint8_t const lun = 0;

	_disk_busy[volume % NumSdCards] = true;
	tuh_msc_write10(dev_addr, lun, buff, sector, (uint16_t)count, disk_io_complete, 0);
	wait_for_disk_io(volume % NumSdCards);
	return RES_OK;
}

DRESULT UsbFlashDrive::DiskIoctl(BYTE cmd, void *buff)
{
	uint8_t const dev_addr = (volume % NumSdCards) + 1;
	uint8_t const lun = 0;
	switch (cmd)
	{
	case CTRL_SYNC:
		// nothing to do since we do blocking
		return RES_OK;

	case GET_SECTOR_COUNT:
		*((DWORD *)buff) = (WORD)tuh_msc_get_block_count(dev_addr, lun);
		return RES_OK;

	case GET_SECTOR_SIZE:
		*((WORD *)buff) = (WORD)tuh_msc_get_block_size(dev_addr, lun);
		return RES_OK;

	case GET_BLOCK_SIZE:
		*((DWORD *)buff) = 1; // erase block size in units of sector size
		return RES_OK;

	default:
		return RES_PARERR;
	}

	return RES_OK;
}

/*static*/ void UsbFlashDrive::UsbInserted(uint8_t address)
{
	// present = true;
	// uint8_t const lun = 0;
	// tuh_msc_inquiry(dev_addr, lun, &inquiry_resp, inquiry_complete_cb, 0);

	for (UsbFlashDrive *drive : usbDrives)
	{
		// Find a device with no address
		if (!drive->IsPresent())
		{
			drive->address = address;
		}
	}
}

/*static*/ void UsbFlashDrive::UsbRemoved(uint8_t address)
{
	// present = true;
	// uint8_t const lun = 0;
	// tuh_msc_inquiry(dev_addr, lun, &inquiry_resp, inquiry_complete_cb, 0);

	for (UsbFlashDrive *drive : usbDrives)
	{
		if (drive->address == address)
		{
			drive->address = 0;
		}
	}
}

extern "C" void tuh_msc_mount_cb(uint8_t address)
{
	UsbFlashDrive::UsbInserted(address);
}

extern "C" void tuh_msc_umount_cb(uint8_t address)
{
	UsbFlashDrive::UsbRemoved(address);
}

/*static*/ UsbFlashDrive *UsbFlashDrive::usbDrives[NumUsbDrives];
