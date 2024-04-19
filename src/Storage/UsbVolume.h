#pragma once

#include <cstdint>

#include <ObjectModel/ObjectModel.h>
#include "StorageVolume.h"

class UsbVolume : public StorageVolume
{
public:
	UsbVolume(const char *id, uint8_t slot) : StorageVolume(id, slot) {}

	void Init() noexcept override;

	void Spin() noexcept override;

	GCodeResult Mount(const StringRef& reply, bool reportSuccess) noexcept override;

	bool IsUseable(const StringRef& reply) const noexcept override;
	bool IsMounted() const noexcept override { return state == State::mounted; }
	bool IsDetected() const noexcept override { return state == State::inserted; }

	uint64_t GetCapacity() const noexcept override;
	uint32_t GetInterfaceSpeed() const noexcept override;

	DRESULT DiskInitialize() noexcept override;
	DRESULT DiskStatus() noexcept override;
	DRESULT DiskRead(BYTE *buff, LBA_t sector, UINT count) noexcept override;
	DRESULT DiskWrite(BYTE const *buff, LBA_t sector, UINT count) noexcept override;
	DRESULT DiskIoctl(BYTE ctrl, void *buff) noexcept override;

	static void VolumeInserted(uint8_t address);
	static void VolumeRemoved(uint8_t address);

private:
	enum class State : uint8_t
	{
		free,
		inserted,
		mounted,
		removed
	};

	uint8_t address;
	uint8_t lun;
	State state;
	BinarySemaphore ioDone;

	static UsbVolume* usbDrives[NumUsbDrives];

	bool AcceptVolume(uint8_t address);
	void FreeVolume();

	void DeviceUnmount() noexcept override;
};

