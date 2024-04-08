
#include <cstdint>

#include <Platform/Platform.h>
#include <Platform/RepRap.h>

#include <TinyUsbInterface.h>

#include <tusb.h>
#include <class/msc/msc_host.h>

#include "UsbVolume.h"

static bool disk_io_complete(uint8_t address, tuh_msc_complete_data_t const *cb_data)
{
	(void) address;
	BinarySemaphore *ioDone = reinterpret_cast<BinarySemaphore*>(cb_data->user_arg);
	ioDone->Give();
	return true;
}

void UsbVolume::Init() noexcept
{
	StorageVolume::Init();
	address = 0;

	for (size_t i = 0; i < NumUsbDrives; i++)
	{
		if (usbDrives[i] == nullptr)
		{
			usbDrives[i] = this;
		}
	}
}

void UsbVolume::Spin() noexcept
{
	if (state == State::removed)
	{
		// InternalUnmount();
		address = 0;
		state = State::free;
	}
}

bool UsbVolume::IsUseable() const noexcept
{
	return CoreUsbIsHostMode();
}

GCodeResult UsbVolume::Mount(const StringRef &reply, bool reportSuccess) noexcept
{
	if (!IsDetected())
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
		reply.printf("Cannot mount SD card %u: no FAT filesystem found on card (EXFAT is not supported)", slot);
		return GCodeResult::error;
	}
	if (mounted != FR_OK)
	{
		reply.printf("Cannot mount SD card %u: code %d", slot, mounted);
		return GCodeResult::error;
	}

	state = State::mounted;

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
	return static_cast<DRESULT>(tuh_msc_mounted(address) ? 0 : STA_NODISK);
}

DRESULT UsbVolume::DiskRead(BYTE *buff, LBA_t sector, UINT count) noexcept
{
	tuh_msc_read10(address, lun, buff, sector, (uint16_t)count, disk_io_complete, reinterpret_cast<uintptr_t>(&ioDone));
	ioDone.Take();
	return RES_OK;
}

DRESULT UsbVolume::DiskWrite(BYTE const *buff, LBA_t sector, UINT count) noexcept
{
	tuh_msc_write10(address, lun, buff, sector, (uint16_t)count, disk_io_complete, reinterpret_cast<uintptr_t>(&ioDone));
	ioDone.Take();
	return RES_OK;
}

DRESULT UsbVolume::DiskIoctl(BYTE cmd, void *buff) noexcept
{
	switch (cmd)
	{
	case CTRL_SYNC:
		// nothing to do since we do blocking
		return RES_OK;

	case GET_SECTOR_COUNT:
		*((DWORD *)buff) = (WORD)tuh_msc_get_block_count(address, lun);
		return RES_OK;

	case GET_SECTOR_SIZE:
		*((WORD *)buff) = (WORD)tuh_msc_get_block_size(address, lun);
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
		state = State::inserted;
		this->address = address;
		return true;
	}
	return false;
}

void UsbVolume::FreeVolume()
{
	if (state == State::inserted)
	{
		state = State::free;
		address = 0;
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
