#pragma once

#include "StorageDevice.h"

class SdCard : public StorageDevice
{
public:

    enum class DetectState : uint8_t
    {
        notPresent = 0,
        inserting,
        present,
        removing
    };

    void Spin() noexcept override;
    GCodeResult Mount(size_t num, const StringRef& reply, bool reportSuccess) noexcept override;
    GCodeResult Unmount(size_t num, const StringRef& reply) noexcept override;

    static SdCard &GetSdCard(uint8_t num);
private:
    void Clear() noexcept;

	uint32_t cdChangedTime;
	Pin cdPin;
	DetectState cardState;
};
