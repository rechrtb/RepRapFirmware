#pragma once

#include <cstdint>

#include <ObjectModel/ObjectModel.h>
#include "StorageVolume.h"

class UsbVolume : public StorageVolume
{
public:
    UsbVolume(const char *id, uint8_t num) : StorageVolume(id, num) {}

    GCodeResult Mount(const StringRef& reply, bool reportSuccess) noexcept override;
    GCodeResult Unmount(const StringRef& reply) noexcept override;

    void Spin() noexcept override;

    uint64_t GetCapacity() const override;
    uint32_t GetInterfaceSpeed() const override;

    void Init() noexcept override;

    DRESULT DiskInitialize() override;
    DRESULT DiskStatus() override;
    DRESULT DiskRead(BYTE *buff, LBA_t sector, UINT count) override;
    DRESULT DiskWrite(BYTE const *buff, LBA_t sector, UINT count) override;
    DRESULT DiskIoctl(BYTE ctrl, void *buff) override;

    bool IsMounted() const noexcept override { return state == State::mounted; }
    bool IsDetected() const noexcept override { return address; }

    static void UsbInserted(uint8_t address);
    static void UsbRemoved(uint8_t address);

private:
    static UsbVolume* usbDrives[NumUsbDrives];
    uint8_t address;
    uint8_t lun { 0 }; // support only one for now

    enum class State : uint8_t
    {
        free,
        inserted,
        mounting,
        mounted,
        removed
    };

    State state;

    bool AcceptDevice(uint8_t address);
    void RemoveDevice();
    unsigned int InternalUnmount() noexcept;
};

