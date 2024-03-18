#pragma once

#include <cstdint>

#include <ObjectModel/ObjectModel.h>
#include "StorageDevice.h"

class UsbFlashDrive : public StorageDevice
{
public:

    UsbFlashDrive(const char *id, uint8_t volume) : StorageDevice(id, volume) {}

    GCodeResult Mount(size_t num, const StringRef& reply, bool reportSuccess) noexcept override { return GCodeResult::error; }
    unsigned int Unmount() noexcept override { return 0; }

    void Init() noexcept override {}

    void Spin() noexcept override {}

    uint64_t GetCapacity() const override;
    uint32_t GetInterfaceSpeed() const override;

private:
};

