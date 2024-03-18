#pragma once

#include <ObjectModel/ObjectModel.h>
#include "StorageDevice.h"

class SdCard : public StorageDevice
{
public:

	enum class InfoResult : uint8_t
	{
		badSlot = 0,
		noCard = 1,
		ok = 2
	};


    SdCard(const char *id, uint8_t volume) : StorageDevice(id, volume) {}

    void Init() noexcept override;

    void Spin() noexcept;
    GCodeResult Mount(size_t num, const StringRef& reply, bool reportSuccess) noexcept override;
    unsigned int Unmount() noexcept override;

    GCodeResult SetCSPin(GCodeBuffer& gb, const StringRef& reply) noexcept;


    uint64_t GetCapacity() const override;
    uint32_t GetInterfaceSpeed() const override;

    bool Useable() noexcept override;
    FATFS* GetFS() noexcept { return &fileSystem; }
    void GetStats(Stats& stats) noexcept;
    void ResetStats() noexcept;

private:
	uint32_t cdChangedTime;
	Pin cdPin;

    void Clear() noexcept;
};
