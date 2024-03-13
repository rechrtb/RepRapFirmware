#include "ff.h"				// for type definitions
#include "diskio.h"
#include "diskio_usbhmsc.h"

DSTATUS disk_usbhmsc_initialize (BYTE pdrv) noexcept
{
    return RES_ERROR;
}

DSTATUS disk_usbhmsc_status (BYTE pdrv) noexcept
{
    return RES_ERROR;
}

DRESULT disk_usbhmsc_read (BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) noexcept
{
    return RES_ERROR;
}

DRESULT disk_usbhmsc_write (BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) noexcept
{
    return RES_ERROR;
}

DRESULT disk_usbhmsc_ioctl (BYTE pdrv, BYTE cmd, void* buff) noexcept
{
    return RES_ERROR;
}