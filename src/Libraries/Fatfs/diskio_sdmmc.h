/*-----------------------------------------------------------------------/
/  Low level disk interface modlue include file   (C)ChaN, 2019          /
/-----------------------------------------------------------------------*/

#ifndef _DISKIO_SDMMC_DEFINED
#define _DISKIO_SDMMC_DEFINED

#ifdef __cplusplus

unsigned int DiskioSdmmcGetAndClearMaxRetryCount() noexcept;
float DiskioSdmmcGetAndClearLongestReadTime() noexcept;
float DiskioSdmmcGetAndClearLongestWriteTime() noexcept;

extern "C" {

#endif

DSTATUS disk_sdmmc_initialize (BYTE pdrv) noexcept;
DSTATUS disk_sdmmc_status (BYTE pdrv) noexcept;
DRESULT disk_sdmmc_read (BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) noexcept;
DRESULT disk_sdmmc_write (BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) noexcept;
DRESULT disk_sdmmc_ioctl (BYTE pdrv, BYTE cmd, void* buff) noexcept;

#ifdef __cplusplus
}
#endif

#endif
