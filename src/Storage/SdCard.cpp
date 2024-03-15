#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <ObjectModel/ObjectModel.h>


#include "SdCard.h"
#include "MassStorage.h"

# if HAS_MASS_STORAGE

# include <Libraries/sd_mmc/sd_mmc.h>
# include <Libraries/sd_mmc/conf_sd_mmc.h>


# if SAME70
alignas(4) static __nocache uint8_t sectorBuffers[NumSdCards][512];
#endif


// Check that the correct number of SD cards is configured in the library
static_assert(SD_MMC_MEM_CNT == NumSdCards);

#ifdef DUET3_MB6HC
static IoPort sd1Ports[2];		// first element is CS port, second is CD port
#endif

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocate in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(SdCard, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(SdCard, _condition,__VA_ARGS__)

// These functions are only called from one place each in the OM table, hence inlined
uint64_t SdCard::GetCapacity() const
{
	return 0;
}

uint64_t SdCard::GetFreeSpace() const
{
	return 0;
}

uint64_t SdCard::GetPartitionSize() const
{
	return 0;
}

double SdCard::GetInterfaceSpeed() const
{
	return 0;
    //return (double)((float)sd_mmc_get_interface_speed(num) * 0.000001);
}

constexpr ObjectModelTableEntry SdCard::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. volumes[] root
	{ "capacity",			OBJECT_MODEL_FUNC_IF(self->isMounted, self->GetCapacity()),								    ObjectModelEntryFlags::none },
	{ "freeSpace",			OBJECT_MODEL_FUNC_IF(self->isMounted, self->GetFreeSpace()),							        ObjectModelEntryFlags::none },
	{ "mounted",			OBJECT_MODEL_FUNC(self->isMounted),														ObjectModelEntryFlags::none },
	{ "openFiles",			OBJECT_MODEL_FUNC_IF(self->isMounted, MassStorage::AnyFileOpen(&(self->fileSystem))),	ObjectModelEntryFlags::none },
	{ "partitionSize",		OBJECT_MODEL_FUNC_IF(self->isMounted, self->GetPartitionSize()),						        ObjectModelEntryFlags::none },
	{ "path",				OBJECT_MODEL_FUNC(self->GetPathName()),										    	ObjectModelEntryFlags::verbose },
	{ "speed",				OBJECT_MODEL_FUNC_IF(self->isMounted, (int32_t)self->GetInterfaceSpeed()),					        	ObjectModelEntryFlags::none },
};

// TODO Add storages here in the format
/*
	openFiles = null
	path = null
*/

constexpr uint8_t SdCard::objectModelTableDescriptor[] = { 1, 7 };

DEFINE_GET_OBJECT_MODEL_TABLE(SdCard)

#endif

static const char* TranslateCardError(sd_mmc_err_t err) noexcept
{
	switch (err)
	{
		case SD_MMC_ERR_NO_CARD:
			return "Card not found";
		case SD_MMC_ERR_UNUSABLE:
			return "Card is unusable";
		case SD_MMC_ERR_SLOT:
			return "num unknown";
		case SD_MMC_ERR_COMM:
			return "Communication error";
		case SD_MMC_ERR_PARAM:
			return "Illegal input parameter";
		case SD_MMC_ERR_WP:
			return "Card write protected";
		default:
			return "Unknown error";
	}
}


static const char* TranslateCardType(card_type_t ct) noexcept
{
	switch (ct)
	{
		case CARD_TYPE_SD | CARD_TYPE_HC:
			return "SDHC";
		case CARD_TYPE_SD:
			return "SD";
		case CARD_TYPE_MMC | CARD_TYPE_HC:
			return "MMC High Capacity";
		case CARD_TYPE_MMC:
			return "MMC";
		case CARD_TYPE_SDIO:
			return "SDIO";
		case CARD_TYPE_SD_COMBO:
			return "SD COMBO";
		case CARD_TYPE_UNKNOWN:
		default:
			return "Unknown type";
	}
}


SdCard::SdCard(const char* id, uint8_t volume)
{
    id = id;
    volume = volume;
    mounting = isMounted = false;
    seq = 0;
    cdPin = SdCardDetectPins[volume];
    cardState = (cdPin == NoPin) ? DetectState::present : DetectState::notPresent;
}

bool SdCard::Useable() noexcept
{
#if DUET3_MB6HC
    if (volume == 1)
    {
        return sd1Ports[0].IsValid();
    }
#endif
    return true;
}

void SdCard::Init(uint8_t num) noexcept
{
    num = num;
    Clear();
    mounting = isMounted = false;
    seq = 0;
    cdPin = SdCardDetectPins[num];
    cardState = (cdPin == NoPin) ? DetectState::present : DetectState::notPresent;
    volMutex.Create("0");

	// sd_mmc_init(SdWriteProtectPins, SdSpiCSPins);		// initialize SD MMC stack
}


GCodeResult SdCard::Mount(size_t num, const StringRef& reply, bool reportSuccess) noexcept
{
    MutexLocker lock1(MassStorage::GetFsMutex());
    MutexLocker lock(volMutex);

    if (cardState == DetectState::notPresent)
    {
        reply.copy("No SD card present");
        mounting = false;
        return GCodeResult::error;
    }

    if (cardState != DetectState::present)
    {
        return GCodeResult::notFinished;						// wait for debounce to finish
    }

    const sd_mmc_err_t err = sd_mmc_check(volume); // usb_host_support: sd card numbering
    if (err != SD_MMC_OK && millis() - mountStartTime < 5000)
    {
        delay(2);
        return GCodeResult::notFinished;
    }

    mounting = false;
    if (err != SD_MMC_OK)
    {
        reply.printf("Cannot initialise SD card %u: %s", volume, TranslateCardError(err));
        return GCodeResult::error;
    }

    // Mount the file systems
    const char path[3] = { static_cast<char>('0' + volume), ':', 0 };
    const FRESULT mounted = f_mount(&fileSystem, path, 1);
    if (mounted == FR_NO_FILESYSTEM)
    {
        reply.printf("Cannot mount SD card %u: no FAT filesystem found on card (EXFAT is not supported)", volume);
        return GCodeResult::error;
    }
    if (mounted != FR_OK)
    {
        reply.printf("Cannot mount SD card %u: code %d", volume, mounted);
        return GCodeResult::error;
    }

    isMounted = true;
    reprap.VolumesUpdated();
    if (reportSuccess)
    {
        float capacity = ((float)sd_mmc_get_capacity(volume) * 1024) / 1000000;		// get capacity and convert from Kib to Mbytes
        const char* capUnits;
        if (capacity >= 1000.0)
        {
            capacity /= 1000;
            capUnits = "Gb";
        }
        else
        {
            capUnits = "Mb";
        }
        reply.printf("%s card mounted in volume %u, capacity %.2f%s", TranslateCardType(sd_mmc_get_type(volume)), volume, (double)capacity, capUnits);
    }

	return GCodeResult::ok;
}


GCodeResult SdCard::SetCSPin(GCodeBuffer& gb, const StringRef& reply) noexcept
{
	IoPort * const portAddresses[2] = { &sd1Ports[0], &sd1Ports[1] };
	if (gb.Seen('C'))
	{
        const PinAccess accessNeeded[2] = { PinAccess::write1, PinAccess::read };
        if (IoPort::AssignPorts(gb, reply, PinUsedBy::sdCard, 2, portAddresses, accessNeeded) == 0)
        {
			return GCodeResult::error;
        }
        sd_mmc_change_cs_pin(1, sd1Ports[0].GetPin());
        cdPin = sd1Ports[1].GetPin();
        if (cdPin == NoPin)
        {
            cardState = DetectState::present;
        }
    }
    else
    {
		IoPort::AppendPinNames(reply, 2, portAddresses);
    }
    reprap.VolumesUpdated();
    return GCodeResult::ok;
}

void SdCard::Spin() noexcept
{
    if (cdPin != NoPin)
    {
        if (IoPort::ReadPin(cdPin))
        {
            // Pin state says no card present
            switch (cardState)
            {
            case DetectState::inserting:
            case DetectState::present:
                cardState = DetectState::removing;
                cdChangedTime = millis();
                break;

            case DetectState::removing:
                if (millis() - cdChangedTime > SdCardDetectDebounceMillis)
                {
                    cardState = DetectState::notPresent;
                    if (isMounted)
                    {
                        const unsigned int numFiles = Unmount();
                        if (numFiles != 0)
                        {
                            reprap.GetPlatform().MessageF(ErrorMessage, "SD card %u removed with %u file(s) open on it\n", volume, numFiles);
                        }
                    }
                }
                break;

            default:
                break;
            }
        }
        else
        {
            // Pin state says card is present
            switch (cardState)
            {
            case DetectState::removing:
            case DetectState::notPresent:
                cardState = DetectState::inserting;
                cdChangedTime = millis();
                break;

            case DetectState::inserting:
                cardState = DetectState::present;
                break;

            default:
                break;
            }
        }
    }
}

void SdCard::Clear() noexcept
{
	memset(&fileSystem, 0, sizeof(fileSystem));
#if SAME70
	fileSystem.win = sectorBuffers[volume];
	memset(sectorBuffers[volume], 0, sizeof(sectorBuffers[volume]));
#endif
}

void SdCard::GetStats(Stats& stats) noexcept
{
    stats = this->stats;
}

void SdCard::ResetStats() noexcept
{
    memset(&stats, 0, sizeof(stats));
}

unsigned int SdCard::Unmount() noexcept
{
	MutexLocker lock1(MassStorage::GetFsMutex());
	MutexLocker lock2(volMutex);
	const unsigned int invalidated = MassStorage::InvalidateFiles(&fileSystem);
	const char path[3] = { (char)('0' + volume), ':', 0 };
	f_mount(nullptr, path, 0);
	Clear();
	sd_mmc_unmount(volume);
	isMounted = false;
	reprap.VolumesUpdated();
	++seq;
	return invalidated;
}

SdCard::InfoResult SdCard::GetInfo(Info& returnedInfo) noexcept
{
	if (!isMounted)
	{
		return InfoResult::noCard;
	}

	returnedInfo.cardCapacity = (uint64_t)sd_mmc_get_capacity(num) * 1024;
	returnedInfo.speed = sd_mmc_get_interface_speed(num);
	String<StringLength50> path;
	path.printf("%u:/", num);
	uint32_t freeClusters;
	FATFS *fs;
	const FRESULT fr = f_getfree(path.c_str(), &freeClusters, &fs);
	if (fr == FR_OK)
	{
		returnedInfo.clSize = fs->csize * 512;
		returnedInfo.partitionSize = (uint64_t)(fs->n_fatent - 2) * returnedInfo.clSize;
		returnedInfo.freeSpace = (uint64_t)freeClusters * returnedInfo.clSize;
	}
	else
	{
		returnedInfo.clSize = 0;
		returnedInfo.cardCapacity = returnedInfo.partitionSize = returnedInfo.freeSpace = 0;
	}
	return InfoResult::ok;
}


bool SdCard::IsPresent() noexcept
{
	return cardState == DetectState::present;
}



# endif
