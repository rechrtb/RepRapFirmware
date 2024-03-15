
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <RepRapFirmware.h>

# include <Libraries/Fatfs/diskio.h>
# include <Libraries/Fatfs/diskio_sdmmc.h>

// Functions called by FatFS to acquire/release mutual exclusion
extern "C"
{
	// Create a sync object. We already created it so just need to return success.
	int ff_mutex_create (int vol) noexcept
	{
		return 1;
	}

	// Lock sync object
	int ff_mutex_take (int vol) noexcept
	{
		if (vol < NumSdCards)
		{
			//info[vol].volMutex.Take();
		}
		else
		{
			//drives[vol % NumSdCards].volMutex.Take();
		}
		return 1;
	}

	// Unlock sync object
	void ff_mutex_give (int vol) noexcept
	{
		if (vol < NumSdCards)
		{
			//info[vol].volMutex.Release();
		}
		else
		{
			//drives[vol % NumSdCards].volMutex.Release();
		}
	}

	// Delete a sync object
	void ff_mutex_delete (int vol) noexcept
	{
		// nothing to do, we never delete the mutex
	}
}
