
#include <cstdint>

#include <Platform/Platform.h>
#include <Platform/RepRap.h>

#include <tusb.h>
#include <class/msc/msc_host.h>

#include "UsbVolume.h"

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

void UsbVolume::Init() noexcept
{
	StorageVolume::Init();
	address = 0;
	usbDrives[num % NumSdCards] = this;
}

void UsbVolume::Spin() noexcept
{
	if (state == State::removed)
	{
		// TODO: Unmount
		address = 0;
		state = State::free;
	}
}

GCodeResult UsbVolume::Mount(const StringRef &reply, bool reportSuccess) noexcept
{
	if (IsDetected())
	{
		reply.copy("No USB storage detected");
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
		reply.printf("Cannot mount SD card %u: no FAT filesystem found on card (EXFAT is not supported)", num);
		return GCodeResult::error;
	}
	if (mounted != FR_OK)
	{
		reply.printf("Cannot mount SD card %u: code %d", num, mounted);
		return GCodeResult::error;
	}

	state = State::mounted;

	return GCodeResult::ok;
}

GCodeResult UsbVolume::Unmount(const StringRef& reply) noexcept
{
	// if (MassStorage::AnyFileOpen(&fileSystem))
	// {
	// 	// Don't unmount the card if any files are open on it
	// 	reply.copy("USB storage has open file(s)");
	// 	return GCodeResult::error;
	// }

	// (void)InternalUnmount();
	// reply.printf("USB storage %u may now be removed", num);
	// ++seqNum;

	// MutexLocker lock1(MassStorage::GetFsMutex());
	// MutexLocker lock2(mutex);
	// const unsigned int invalidated = MassStorage::InvalidateFiles(&fileSystem);
	// f_mount(nullptr, path, 0);
	// Clear();
	// //sd_mmc_unmount(num);
	// reprap.VolumesUpdated();
	// return invalidated;

	return GCodeResult::ok;
}

uint64_t UsbVolume::GetCapacity() const noexcept
{
	// Get capacity of device
	uint32_t const block_count = tuh_msc_get_block_count(address, lun);
	uint32_t const block_size = tuh_msc_get_block_size(address, lun);
	return (block_count * block_size) / (1024);
}

uint32_t UsbVolume::GetInterfaceSpeed() const noexcept
{
	tusb_speed_t speed = tuh_speed_get(address);
	return (speed == TUSB_SPEED_HIGH ? 480000000 : 12000000) / 8;
}


DRESULT UsbVolume::DiskInitialize() noexcept
{
	return RES_OK; // nothing to do
}

DRESULT UsbVolume::DiskStatus() noexcept
{
	uint8_t dev_addr = (num % NumSdCards) + 1;
	return static_cast<DRESULT>(tuh_msc_mounted(dev_addr) ? 0 : STA_NODISK);
}

DRESULT UsbVolume::DiskRead(BYTE *buff, LBA_t sector, UINT count) noexcept
{
	uint8_t const dev_addr = (num % NumSdCards) + 1;
	uint8_t const lun = 0;

	_disk_busy[num % NumSdCards] = true;
	tuh_msc_read10(dev_addr, lun, buff, sector, (uint16_t)count, disk_io_complete, 0);
	wait_for_disk_io(num % NumSdCards);

	return RES_OK;
}

DRESULT UsbVolume::DiskWrite(BYTE const *buff, LBA_t sector, UINT count) noexcept
{
	uint8_t const dev_addr = (num % NumSdCards) + 1;
	uint8_t const lun = 0;

	_disk_busy[num % NumSdCards] = true;
	tuh_msc_write10(dev_addr, lun, buff, sector, (uint16_t)count, disk_io_complete, 0);
	wait_for_disk_io(num % NumSdCards);
	return RES_OK;
}

DRESULT UsbVolume::DiskIoctl(BYTE cmd, void *buff) noexcept
{
	uint8_t const dev_addr = (num % NumSdCards) + 1;
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

/*static*/ void UsbVolume::VolumeInserted(uint8_t address)
{
	for (UsbVolume *drive : usbDrives)
	{
		// Check if there are free ones that can accept
		if (drive->AcceptVolume(address))
		{
			break;
		}
	}
}

/*static*/ void UsbVolume::VolumeRemoved(uint8_t address)
{

	for (UsbVolume *drive : usbDrives)
	{
		if (drive->address == address)
		{
			drive->FreeVolume();
		}
	}
}

bool UsbVolume::AcceptVolume(uint8_t address)
{
	if (state == State::free)
	{
		this->address = address;
		return true;
	}
	return false;
}

void UsbVolume::FreeVolume()
{
	if (state == State::inserted)
	{
		address = 0;
		state = State::free;
	}
	else if (state == State::mounted)
	{
		// Can't free here, must be unmounted and freed in the spin function.
		state = State::removed;
	}
}

extern "C" void tuh_msc_mount_cb(uint8_t address)
{
	UsbVolume::VolumeInserted(address);
}

extern "C" void tuh_msc_umount_cb(uint8_t address)
{
	UsbVolume::VolumeRemoved(address);
}

/*static*/ UsbVolume *UsbVolume::usbDrives[NumUsbDrives];
