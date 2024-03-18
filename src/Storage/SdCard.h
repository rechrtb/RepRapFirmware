#pragma once

#include <ObjectModel/ObjectModel.h>
#include "StorageDevice.h"

class SdCard INHERIT_OBJECT_MODEL
{
public:

    struct Stats
    {
        uint32_t maxReadTime;
        uint32_t maxWriteTime;
        uint32_t maxRetryCount;
    };

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

    SdCard(const char *id, uint8_t volume);

    void Init() noexcept;

    void Spin() noexcept;
    GCodeResult Mount(size_t num, const StringRef& reply, bool reportSuccess) noexcept;
    unsigned int Unmount() noexcept;

    GCodeResult SetCSPin(GCodeBuffer& gb, const StringRef& reply) noexcept;

    bool IsPresent() noexcept;
    bool IsMounted() noexcept { return isMounted; }

    uint64_t GetCapacity() const;
    uint64_t GetFreeSpace() const;
    uint64_t GetPartitionSize() const;
    uint64_t GetClusterSize() const;
    uint32_t GetInterfaceSpeed() const;

    bool Useable() noexcept;
    FATFS* GetFS() noexcept { return &fileSystem; }
    void GetStats(Stats& stats) noexcept;
    void ResetStats() noexcept;

    int GetSequenceNum() noexcept { return seq; }

    void IncrementSeq() noexcept { ++seq; }

    const char* GetPathName() noexcept { return path; }

    Mutex& GetMutex() { return volMutex; }

protected:
	DECLARE_OBJECT_MODEL

private:
    const char *id;
    uint8_t volume;

    char path[3] = "0:";

	FATFS fileSystem;
	uint32_t mountStartTime;
	Mutex volMutex;
	uint16_t seq;
	bool mounting;
	bool isMounted;
    uint8_t num;

    Stats stats;

    void Clear() noexcept;

	uint32_t cdChangedTime;
	Pin cdPin;
	DetectState cardState;


};
