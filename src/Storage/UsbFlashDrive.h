#pragma once

#include <cstdint>

#include <ObjectModel/ObjectModel.h>
#include "StorageDevice.h"

class UsbFlashDrive : public StorageDevice
{
public:
    UsbFlashDrive(const char *id, uint8_t volume) : StorageDevice(id, volume) {}

    GCodeResult Mount(size_t num, const StringRef& reply, bool reportSuccess) noexcept override;
    unsigned int Unmount() noexcept override;

    void Spin() noexcept override;

    uint64_t GetCapacity() const override;
    uint32_t GetInterfaceSpeed() const override;

    DRESULT DiskInitialize() override;
    DRESULT DiskStatus() override;
    DRESULT DiskRead(BYTE *buff, LBA_t sector, UINT count) override;
    DRESULT DiskWrite(BYTE const *buff, LBA_t sector, UINT count) override;
    DRESULT DiskIoctl(BYTE ctrl, void *buff) override;

private:
};

