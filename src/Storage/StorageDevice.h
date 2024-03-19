#pragma once

#include <stdint.h>

#include <ObjectModel/ObjectModel.h>
#include <Libraries/Fatfs/diskio.h>

#include "StorageDevice.h"

class StorageDevice INHERIT_OBJECT_MODEL
{

public:
    struct Stats
    {
        uint32_t maxReadTime;
        uint32_t maxWriteTime;
        uint32_t maxRetryCount;
    };


    StorageDevice(const char *id, uint8_t volume);

    virtual GCodeResult Mount(size_t num, const StringRef& reply, bool reportSuccess) noexcept = 0;
    virtual unsigned int Unmount() noexcept = 0;
    virtual void Spin() noexcept = 0;

    virtual void Init() noexcept;

    FATFS* GetFS() noexcept { return &fileSystem; }
    int GetSequenceNum() noexcept { return seqNum; }
    void IncrementSeq() noexcept { ++seqNum; }
    const char* GetPathName() noexcept { return path; }
    Mutex& GetMutex() { return volMutex; }
    virtual bool Useable() noexcept { return true; }
    bool IsMounted() noexcept { return isMounted; }

    virtual bool IsPresent() noexcept  = 0;

    virtual uint64_t GetCapacity() const = 0;
    virtual uint64_t GetFreeSpace() const;
    virtual uint64_t GetPartitionSize() const;
    virtual uint64_t GetClusterSize() const;
    virtual uint32_t GetInterfaceSpeed() const = 0;

    virtual DRESULT DiskInitialize() = 0;
    virtual DRESULT DiskStatus() = 0;
    virtual DRESULT DiskRead(BYTE *buff, LBA_t sector, UINT count) = 0;
    virtual DRESULT DiskWrite(BYTE const *buff, LBA_t sector, UINT count) = 0;
    virtual DRESULT DiskIoctl(BYTE ctrl, void *buff) = 0;

protected:
	DECLARE_OBJECT_MODEL

    const char *id;
    uint8_t volume;

	Mutex volMutex;

	bool mounting;
	bool isMounted;
	uint32_t mountStartTime;
	uint16_t seqNum;

    Stats stats;
	FATFS fileSystem;

    char path[3] = "0:";
};
