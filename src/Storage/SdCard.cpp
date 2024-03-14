#include "SdCard.h"

# if HAS_MASS_STORAGE

# include <Libraries/sd_mmc/sd_mmc.h>
# include <Libraries/sd_mmc/conf_sd_mmc.h>


static const char* TranslateCardError(sd_mmc_err_t err) noexcept
{
	switch (err)
	{
		case SD_MMC_ERR_NO_CARD:
			return "Card not found";
		case SD_MMC_ERR_UNUSABLE:
			return "Card is unusable";
		case SD_MMC_ERR_SLOT:
			return "Slot unknown";
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

GCodeResult SdCard::Mount(size_t num, const StringRef& reply, bool reportSuccess) noexcept
{
    // SdCardInfo& inf = info[card];
    // MutexLocker lock1(fsMutex);
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

    const sd_mmc_err_t err = sd_mmc_check(num); // usb_host_support: sd card numbering
    if (err != SD_MMC_OK && millis() - mountStartTime < 5000)
    {
        delay(2);
        return GCodeResult::notFinished;
    }

    mounting = false;
    if (err != SD_MMC_OK)
    {
        reply.printf("Cannot initialise SD card %u: %s", num, TranslateCardError(err));
        return GCodeResult::error;
    }

    // Mount the file systems
    const char path[3] = { (char)('0' + num), ':', 0 };
    const FRESULT mounted = f_mount(&fileSystem, path, 1);
    if (mounted == FR_NO_FILESYSTEM)
    {
        reply.printf("Cannot mount SD card %u: no FAT filesystem found on card (EXFAT is not supported)", num);
        return GCodeResult::error;
    }
    if (mounted != FR_OK)
    {
        reply.printf("Cannot mount SD card %u: code %d", num, mounted);
        return GCodeResult::error;
    }

    isMounted = true;
    reprap.VolumesUpdated();
    if (reportSuccess)
    {
        float capacity = ((float)sd_mmc_get_capacity(num) * 1024) / 1000000;		// get capacity and convert from Kib to Mbytes
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
        reply.printf("%s card mounted in slot %u, capacity %.2f%s", TranslateCardType(sd_mmc_get_type(num)), num, (double)capacity, capUnits);
    }

    ++seq;
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
                        // const unsigned int numFiles = InternalUnmount(card);
                        // if (numFiles != 0)
                        // {
                        //     reprap.GetPlatform().MessageF(ErrorMessage, "SD card %u removed with %u file(s) open on it\n", card, numFiles);
                        // }
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

}

GCodeResult SdCard::Unmount(size_t num, const StringRef& reply) noexcept
{
	MutexLocker lock(volMutex);
	const char path[3] = { (char)('0' + num), ':', 0 };
	f_mount(nullptr, path, 0);
	Clear();
	sd_mmc_unmount(num);
	isMounted = false;
	reprap.VolumesUpdated();
    return GCodeResult::ok;
}

# endif
