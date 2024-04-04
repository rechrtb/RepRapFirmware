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

    void Init() noexcept override;

    DRESULT DiskInitialize() override;
    DRESULT DiskStatus() override;
    DRESULT DiskRead(BYTE *buff, LBA_t sector, UINT count) override;
    DRESULT DiskWrite(BYTE const *buff, LBA_t sector, UINT count) override;
    DRESULT DiskIoctl(BYTE ctrl, void *buff) override;


    bool IsPresent() noexcept override { return address; }

    static void UsbInserted(uint8_t address);
    static void UsbRemoved(uint8_t address);
private:
    static UsbFlashDrive* usbDrives[NumUsbDrives];
    uint8_t address;
    uint8_t lun { 0 }; // support only one for now
};

