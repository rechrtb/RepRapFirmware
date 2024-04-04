#include "StorageVolume.h"
#include "MassStorage.h"

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(StorageVolume, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...) OBJECT_MODEL_FUNC_IF_BODY(StorageVolume, _condition, __VA_ARGS__)

constexpr ObjectModelTableEntry StorageVolume::objectModelTable[] =
	{
		// Within each group, these entries must be in alphabetical order
		// 0. volumes[] root
		{"capacity", OBJECT_MODEL_FUNC_IF(self->IsMounted(), self->GetCapacity()), ObjectModelEntryFlags::none},
		{"freeSpace", OBJECT_MODEL_FUNC_IF(self->IsMounted(), self->GetFreeSpace()), ObjectModelEntryFlags::none},
		{"mounted", OBJECT_MODEL_FUNC(self->IsMounted()), ObjectModelEntryFlags::none},
		{"openFiles", OBJECT_MODEL_FUNC_IF(self->IsMounted(), MassStorage::AnyFileOpen(&(self->fileSystem))), ObjectModelEntryFlags::none},
		{"partitionSize", OBJECT_MODEL_FUNC_IF(self->IsMounted(), self->GetPartitionSize()), ObjectModelEntryFlags::none},
		{"path", OBJECT_MODEL_FUNC(self->path), ObjectModelEntryFlags::verbose},
		{"speed", OBJECT_MODEL_FUNC_IF(self->IsMounted(), (int32_t)self->GetInterfaceSpeed()), ObjectModelEntryFlags::none},
};

// TODO Add storages here in the format
/*
	openFiles = null
	path = null
*/

constexpr uint8_t StorageVolume::objectModelTableDescriptor[] = {1, 7};

DEFINE_GET_OBJECT_MODEL_TABLE(StorageVolume)

#endif

# if SAME70
alignas(4) static __nocache uint8_t sectorBuffers[FF_VOLUMES][FF_MAX_SS];
#endif

void StorageVolume::Clear()
{
	memset(&fileSystem, 0, sizeof(fileSystem));
#if SAME70
	fileSystem.win = sectorBuffers[num];
	memset(sectorBuffers[num], 0, sizeof(sectorBuffers[num]));
#endif
}

uint64_t StorageVolume::GetFreeSpace() const
{
	uint64_t res = fileSystem.free_clst * GetClusterSize();
	if (res < GetPartitionSize())
	{
		return res;
	}
	return 0; // free_clst is not valid, full FAT scan might not be worth it (such as on rare FAT16/FAT12 drives)
}

uint64_t StorageVolume::GetPartitionSize() const
{
	return (fileSystem.n_fatent - 2) * GetClusterSize();
}

void StorageVolume::Init() noexcept
{
	memset(&fileSystem, 0, sizeof(fileSystem));
	seqNum = 0;
	mutex.Create(id);
}

uint64_t StorageVolume::GetClusterSize() const
{
	return (fileSystem.csize) * 512;
}

StorageVolume::StorageVolume(const char *id, uint8_t num)
{
	this->id = id;
	this->num = num;
	path[0] += num;
}
