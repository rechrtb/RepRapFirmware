#pragma once

#include "StorageDevice.h"

class SdCard
{
public:
    struct Info
    {
        uint64_t cardCapacity;
        uint64_t partitionSize;
        uint64_t freeSpace;
        uint32_t clSize;
        uint32_t speed;
    };

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

    void Init(uint8_t num) noexcept;

    void Spin() noexcept;
    GCodeResult Mount(size_t num, const StringRef& reply, bool reportSuccess) noexcept;
    unsigned int Unmount() noexcept;

    GCodeResult SetCSPin(GCodeBuffer& gb, const StringRef& reply) noexcept;

    bool IsPresent() noexcept;
    double GetInterfaceSpeed() noexcept;

    bool Useable() noexcept;

    InfoResult GetInfo(Info& info) noexcept;

    void GetStats(Stats& stats) noexcept;
    void ResetStats() noexcept;

private:
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
