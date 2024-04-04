#pragma once

#include <ObjectModel/ObjectModel.h>
#include "StorageVolume.h"

class SdCardVolume : public StorageVolume
{
public:

	struct Stats
	{
		uint32_t maxReadTime;
		uint32_t maxWriteTime;
		uint32_t maxRetryCount;
	};

	enum class InfoResult : uint8_t
	{
		badSlot = 0,
		noCard = 1,
		ok = 2
	};

	SdCardVolume(const char *id, uint8_t num) : StorageVolume(id, num) {}

	void Init() noexcept override;

	void Spin() noexcept override;

	GCodeResult Mount(const StringRef& reply, bool reportSuccess) noexcept override;
	GCodeResult Unmount(const StringRef& reply) noexcept override;

	bool Useable() const noexcept override;
	bool IsMounted() const noexcept override { return isMounted; }
	bool IsDetected() const noexcept override { return detectState == DetectState::present; }

	uint64_t GetCapacity() const noexcept override;
	uint32_t GetInterfaceSpeed() const noexcept override;

	DRESULT DiskInitialize() noexcept override;
	DRESULT DiskStatus() noexcept override;
	DRESULT DiskRead(BYTE *buff, LBA_t sector, UINT count) noexcept override;
	DRESULT DiskWrite(BYTE const *buff, LBA_t sector, UINT count) noexcept override;
	DRESULT DiskIoctl(BYTE ctrl, void *buff) noexcept override;

	GCodeResult SetCSPin(GCodeBuffer& gb, const StringRef& reply) noexcept;

	static Stats GetStats() noexcept;
	static void ResetStats() noexcept;

private:
	enum class DetectState : uint8_t
	{
		notPresent = 0,
		inserting,
		present,
		removing
	};

	bool mounting;
	bool isMounted;
	uint32_t mountStartTime;
	uint32_t cdChangedTime;
	DetectState detectState;
	Pin cdPin;

	static Stats stats;

	unsigned int InternalUnmount() noexcept;
};
