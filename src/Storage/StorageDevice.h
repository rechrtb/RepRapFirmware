#pragma once

#include <stdint.h>

#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <ObjectModel/ObjectModel.h>

class StorageDevice
{
    struct Info
    {
        uint64_t capacity;
        uint64_t partitionSize;
        uint64_t freeSpace;
        uint32_t clusterSize;
        uint32_t speed;
    };

    void GetInfo(StorageDevice::Info& info);
    virtual void Spin() noexcept = 0;
    virtual GCodeResult Mount(size_t num, const StringRef& reply, bool reportSuccess) noexcept = 0;
    virtual GCodeResult Unmount(size_t num, const StringRef& reply) noexcept = 0;
public:
protected:
	FATFS fileSystem;
	uint32_t mountStartTime;
	Mutex volMutex;
	uint16_t seq;
	bool mounting;
	bool isMounted;
};