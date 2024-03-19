#include "StorageDevice.h"
#include "MassStorage.h"

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(StorageDevice, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...) OBJECT_MODEL_FUNC_IF_BODY(StorageDevice, _condition, __VA_ARGS__)

constexpr ObjectModelTableEntry StorageDevice::objectModelTable[] =
	{
		// Within each group, these entries must be in alphabetical order
		// 0. volumes[] root
		{"capacity", OBJECT_MODEL_FUNC_IF(self->isMounted, self->GetCapacity()), ObjectModelEntryFlags::none},
		{"freeSpace", OBJECT_MODEL_FUNC_IF(self->isMounted, self->GetFreeSpace()), ObjectModelEntryFlags::none},
		{"mounted", OBJECT_MODEL_FUNC(self->isMounted), ObjectModelEntryFlags::none},
		{"openFiles", OBJECT_MODEL_FUNC_IF(self->isMounted, MassStorage::AnyFileOpen(&(self->fileSystem))), ObjectModelEntryFlags::none},
		{"partitionSize", OBJECT_MODEL_FUNC_IF(self->isMounted, self->GetPartitionSize()), ObjectModelEntryFlags::none},
		{"path", OBJECT_MODEL_FUNC(self->path), ObjectModelEntryFlags::verbose},
		{"speed", OBJECT_MODEL_FUNC_IF(self->isMounted, (int32_t)self->GetInterfaceSpeed()), ObjectModelEntryFlags::none},
};

// TODO Add storages here in the format
/*
	openFiles = null
	path = null
*/

constexpr uint8_t StorageDevice::objectModelTableDescriptor[] = {1, 7};

DEFINE_GET_OBJECT_MODEL_TABLE(StorageDevice)

#endif

uint64_t StorageDevice::GetFreeSpace() const
{
	return fileSystem.free_clst * GetClusterSize();
}

uint64_t StorageDevice::GetPartitionSize() const
{
	return (fileSystem.n_fatent - 2) * GetClusterSize();
}

void StorageDevice::Init() noexcept
{
	memset(&fileSystem, 0, sizeof(fileSystem));
	mounting = isMounted = false;
	seqNum = 0;
	detectState = DetectState::notPresent;
	volMutex.Create(id);
}

uint64_t StorageDevice::GetClusterSize() const
{
	return (fileSystem.csize) * 512;
}

StorageDevice::StorageDevice(const char *id, uint8_t volume)
{
	id = id;
	volume = volume;
	path[0] += volume;
}
