#pragma once

#include <stdint.h>

#include <ObjectModel/ObjectModel.h>
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

    enum class DetectState : uint8_t
    {
        notPresent = 0,
        inserting,
        present,
        removing
    };

    StorageDevice(const char *id, uint8_t volume);

    virtual GCodeResult Mount(size_t num, const StringRef& reply, bool reportSuccess) noexcept = 0;
    virtual unsigned int Unmount() noexcept = 0;

    virtual void Init() noexcept = 0;

    FATFS* GetFS() noexcept { return &fileSystem; }
    int GetSequenceNum() noexcept { return seq; }
    void IncrementSeq() noexcept { ++seq; }
    const char* GetPathName() noexcept { return path; }
    Mutex& GetMutex() { return volMutex; }
    virtual bool Useable() noexcept { return true; }
    bool IsMounted() noexcept { return isMounted; }

    bool IsPresent() noexcept { return cardState == DetectState::present; }

    virtual uint64_t GetCapacity() const = 0;
    virtual uint64_t GetFreeSpace() const;
    virtual uint64_t GetPartitionSize() const;
    virtual uint64_t GetClusterSize() const;
    virtual uint32_t GetInterfaceSpeed() const = 0;

protected:
	DECLARE_OBJECT_MODEL

    const char *id;
    uint8_t volume;

	uint32_t mountStartTime;
	bool mounting;
	uint16_t seq;

	bool isMounted;
	Mutex volMutex;

    Stats stats;

	FATFS fileSystem;

	DetectState cardState;

    char path[3] = "0:";
};
