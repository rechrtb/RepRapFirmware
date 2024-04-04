#pragma once

#include <ObjectModel/ObjectModel.h>
#include "StorageVolume.h"

class SdCardVolume : public StorageVolume
{
public:

	enum class InfoResult : uint8_t
	{
		badSlot = 0,
		noCard = 1,
		ok = 2
	};

    enum class DetectState : uint8_t
    {
        notPresent = 0,
        inserting,
        present,
        removing
    };

    SdCardVolume(const char *id, uint8_t volume) : StorageVolume(id, volume) {}

    void Init() noexcept override;

    void Spin() noexcept override;
    GCodeResult Mount(size_t num, const StringRef& reply, bool reportSuccess) noexcept override;
    GCodeResult Unmount(size_t card, const StringRef& reply) noexcept override;

    GCodeResult SetCSPin(GCodeBuffer& gb, const StringRef& reply) noexcept;
    bool IsPresent() noexcept override { return detectState == DetectState::present; }

    uint64_t GetCapacity() const override;
    uint32_t GetInterfaceSpeed() const override;

    bool Useable() noexcept override;
    void GetStats(Stats& stats) noexcept;
    void ResetStats() noexcept;

    DRESULT DiskInitialize() override;
    DRESULT DiskStatus() override;
    DRESULT DiskRead(BYTE *buff, LBA_t sector, UINT count) override;
    DRESULT DiskWrite(BYTE const *buff, LBA_t sector, UINT count) override;
    DRESULT DiskIoctl(BYTE ctrl, void *buff) override;

private:
	uint32_t cdChangedTime;
	DetectState detectState;
	Pin cdPin;

    unsigned int InternalUnmount() noexcept;
};
