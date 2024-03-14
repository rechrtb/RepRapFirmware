#include <ecv_duet3d.h>

#include <RepRapFirmware.h>

#include "ff.h"				// for type definitions
#include "diskio.h"
#include "diskio_sdmmc.h"

DSTATUS disk_initialize(BYTE drv) noexcept
{
	return (drv < NumSdCards) ? disk_sdmmc_initialize(drv) : RES_ERROR;
}

DSTATUS disk_status(BYTE drv) noexcept
{
	return (drv < NumSdCards) ? disk_sdmmc_status(drv) : RES_ERROR;
}

DRESULT disk_read(BYTE drv, BYTE *buff, LBA_t sector, UINT count) noexcept
{
	return (drv < NumSdCards) ? disk_sdmmc_read(drv, buff, sector, count) : RES_ERROR;
}

#if _READONLY == 0
DRESULT disk_write(BYTE drv, BYTE const *buff, LBA_t sector, UINT count) noexcept
{
	return (drv < NumSdCards) ? disk_sdmmc_write(drv, buff, sector, count) : RES_ERROR;
}

#endif /* _READONLY */

DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void *buff) noexcept
{
	return (drv < NumSdCards) ? disk_sdmmc_ioctl(drv, ctrl, buff) : RES_ERROR;
}
//@}
