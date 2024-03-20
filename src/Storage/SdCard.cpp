#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <ObjectModel/ObjectModel.h>
#include <Movement/StepTimer.h>


#include "SdCard.h"
#include "MassStorage.h"

# if HAS_MASS_STORAGE

#include <Libraries/Fatfs/ff.h> // for type definitions

#include <Libraries/sd_mmc/sd_mmc.h>
#include <Libraries/sd_mmc/conf_sd_mmc.h>
#include <Libraries/sd_mmc/ctrl_access.h>
#include <Libraries/sd_mmc/conf_sd_mmc.h>


// Check that the correct number of SD cards is configured in the library
static_assert(SD_MMC_MEM_CNT == NumSdCards);

#ifdef DUET3_MB6HC
static IoPort sd1Ports[2];		// first element is CS port, second is CD port
#endif

//void debugPrintf(const char*, ...);

//#if (SAM3S || SAM3U || SAM3N || SAM3XA_SERIES || SAM4S)
//# include "rtc.h"
//#endif

/**
 * \defgroup thirdparty_fatfs_port_group Port of low level driver for FatFS
 *
 * Low level driver for FatFS. The driver is based on the ctrl access module
 * of the specific MCU device.
 *
 * @{
 */

/** Default sector size */
#define SECTOR_SIZE_DEFAULT 512

/** Supported sector size. These values are based on the LUN function:
 * mem_sector_size(). */
#define SECTOR_SIZE_512   1
#define SECTOR_SIZE_1024 2
#define SECTOR_SIZE_2048 4
#define SECTOR_SIZE_4096 8

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

bool SdCard::Useable() noexcept
{
// #if DUET3_MB6HC
//     if (volume == 1)
//     {
//         return sd1Ports[0].IsValid();
//     }
// #endif
    return true;
}

void SdCard::Init() noexcept
{
	StorageDevice::Init();
	detectState = (cdPin == NoPin) ? DetectState::present : DetectState::notPresent;
    cdPin = SdCardDetectPins[volume];
    if (volume == 0) // Initialize SD MMC stack on main SD
    {
        sd_mmc_init(SdWriteProtectPins, SdSpiCSPins);
    }
}

GCodeResult SdCard::Mount(size_t volume, const StringRef& reply, bool reportSuccess) noexcept
{
    MutexLocker lock1(MassStorage::GetFsMutex());
    MutexLocker lock(volMutex);

    if (detectState == DetectState::notPresent)
    {
        reply.copy("No SD card present");
        mounting = false;
        return GCodeResult::error;
    }

    if (detectState != DetectState::present)
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
            detectState = DetectState::present;
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
            switch (detectState)
            {
            case DetectState::inserting:
            case DetectState::present:
                detectState = DetectState::removing;
                cdChangedTime = millis();
                break;

            case DetectState::removing:
                if (millis() - cdChangedTime > SdCardDetectDebounceMillis)
                {
                    detectState = DetectState::notPresent;
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
            switch (detectState)
            {
            case DetectState::removing:
            case DetectState::notPresent:
                detectState = DetectState::inserting;
                cdChangedTime = millis();
                break;

            case DetectState::inserting:
                detectState = DetectState::present;
                break;

            default:
                break;
            }
        }
    }
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
	f_mount(nullptr, path, 0);
	memset(&fileSystem, 0, sizeof(fileSystem));
	sd_mmc_unmount(volume);
	isMounted = false;
	reprap.VolumesUpdated();
	++seqNum;
	return invalidated;
}

uint64_t SdCard::GetCapacity() const
{
    return sd_mmc_get_capacity(volume) * 1024;
}

uint32_t SdCard::GetInterfaceSpeed() const
{
	return sd_mmc_get_interface_speed(volume);
}


DRESULT SdCard::DiskInitialize()
{
	if (volume > MAX_LUN) {
		/* At least one of the LUN should be defined */
		return static_cast<DRESULT>(STA_NOINIT);
	}

	Ctrl_status mem_status;

	/* Check LUN ready (USB disk report CTRL_BUSY then CTRL_GOOD) */
	for (int i = 0; i < 2; i ++) {
		mem_status = mem_test_unit_ready(volume);
		if (CTRL_BUSY != mem_status) {
			break;
		}
	}
	if (mem_status != CTRL_GOOD) {
		return static_cast<DRESULT>(STA_NOINIT);
	}

	/* Check Write Protection Status */
	if (mem_wr_protect(volume)) {
		return static_cast<DRESULT>(STA_PROTECT);
	}

	/* The memory should already be initialized */
	return RES_OK;
}

DRESULT SdCard::DiskStatus()
{
	switch (mem_test_unit_ready(volume)) {
	case CTRL_GOOD:
		return RES_OK;
	case CTRL_NO_PRESENT:
		return static_cast<DRESULT>(STA_NOINIT | STA_NODISK);
	default:
		return static_cast<DRESULT>(STA_NOINIT);
	}
}

DRESULT SdCard::DiskRead(BYTE *buff, LBA_t sector, UINT count)
{
	if (reprap.Debug(Module::Storage))
	{
		debugPrintf("Read %u %u %lu\n", volume, count, sector);
	}

	const uint8_t uc_sector_size = mem_sector_size(volume);
	if (uc_sector_size == 0)
	{
		return RES_ERROR;
	}

	/* Check valid address */
	uint32_t ul_last_sector_num;
	mem_read_capacity(volume, &ul_last_sector_num);
	if ((sector + count * uc_sector_size) > (ul_last_sector_num + 1) * uc_sector_size)
	{
		return RES_PARERR;
	}

	/* Read the data */
	unsigned int retryNumber = 0;
	uint32_t retryDelay = SdCardRetryDelay;
	for (;;)
	{
		uint32_t time = StepTimer::GetTimerTicks();
		const Ctrl_status ret = memory_2_ram(volume, sector, buff, count);
		time = StepTimer::GetTimerTicks() - time;
		if (time > stats.maxReadTime)
		{
			stats.maxReadTime = time;
		}

		if (ret == CTRL_GOOD)
		{
			break;
		}

		if (reprap.Debug(Module::Storage))
		{
			debugPrintf("SD read error %d\n", (int)ret);
		}

		++retryNumber;
		if (retryNumber == MaxSdCardTries)
		{
			return RES_ERROR;
		}
		delay(retryDelay);
		retryDelay *= 2;
	}

	if (retryNumber > stats.maxRetryCount)
	{
		stats.maxRetryCount = retryNumber;
	}

	return RES_OK;
}

DRESULT SdCard::DiskWrite(BYTE const *buff, LBA_t sector, UINT count)
{
	if (reprap.Debug(Module::Storage))
	{
		debugPrintf("Write %u %u %lu\n", volume, count, sector);
	}

	const uint8_t uc_sector_size = mem_sector_size(volume);

	if (uc_sector_size == 0)
	{
		return RES_ERROR;
	}

	// Check valid address
	uint32_t ul_last_sector_num;
	mem_read_capacity(volume, &ul_last_sector_num);
	if ((sector + count * uc_sector_size) > (ul_last_sector_num + 1) * uc_sector_size)
	{
		return RES_PARERR;
	}

	// Write the data

	unsigned int retryNumber = 0;
	uint32_t retryDelay = SdCardRetryDelay;
	for (;;)
	{
		uint32_t time = StepTimer::GetTimerTicks();
		const Ctrl_status ret = ram_2_memory(volume, sector, buff, count);
		time = StepTimer::GetTimerTicks() - time;
		if (time > stats.maxWriteTime)
		{
			stats.maxWriteTime = time;
		}

		if (ret == CTRL_GOOD)
		{
			break;
		}

		if (reprap.Debug(Module::Storage))
		{
			debugPrintf("SD write error %d\n", (int)ret);
		}

		++retryNumber;
		if (retryNumber == MaxSdCardTries)
		{
			return RES_ERROR;
		}
		delay(retryDelay);
		retryDelay *= 2;
	}

	if (retryNumber > stats.maxRetryCount)
	{
		stats.maxRetryCount = retryNumber;
	}

	return RES_OK;
}

DRESULT SdCard::DiskIoctl(BYTE ctrl, void *buff)
{
	DRESULT res = RES_PARERR;

	switch (ctrl) {
	case GET_BLOCK_SIZE:
		*(DWORD *)buff = 1;
		res = RES_OK;
		break;

	/* Get the number of sectors on the disk (DWORD) */
	case GET_SECTOR_COUNT:
	{
		uint32_t ul_last_sector_num;

		/* Check valid address */
		mem_read_capacity(volume, &ul_last_sector_num);

		*(DWORD *)buff = ul_last_sector_num + 1;

		res = RES_OK;
	}
	break;

	/* Get sectors on the disk (WORD) */
	case GET_SECTOR_SIZE:
	{
		uint8_t uc_sector_size = mem_sector_size(volume);

		if ((uc_sector_size != SECTOR_SIZE_512) &&
				(uc_sector_size != SECTOR_SIZE_1024) &&
				(uc_sector_size != SECTOR_SIZE_2048) &&
				(uc_sector_size != SECTOR_SIZE_4096)) {
			/* The sector size is not supported by the FatFS */
			return RES_ERROR;
		}

		*(uint8_t *)buff = uc_sector_size * SECTOR_SIZE_DEFAULT;

		res = RES_OK;
	}
	break;

	/* Make sure that data has been written */
	case CTRL_SYNC:
		{
			if (mem_test_unit_ready(volume) == CTRL_GOOD) {
				res = RES_OK;
			} else {
				res = RES_NOTRDY;
			}
		}
		break;

	default:
		res = RES_PARERR;
		break;
	}

	return res;
}

# endif
