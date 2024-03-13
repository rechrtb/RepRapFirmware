#ifndef _DISKIO_USBHMSC_DEFINED
#define _DISKIO_USBHMSC_DEFINED

#ifdef __cplusplus

unsigned int DiskioSdmmcGetAndClearMaxRetryCount() noexcept;
float DiskioSdmmcGetAndClearLongestReadTime() noexcept;
float DiskioSdmmcGetAndClearLongestWriteTime() noexcept;


extern "C" {

#endif


DSTATUS disk_usbhmsc_initialize (BYTE pdrv) noexcept;
DSTATUS disk_usbhmsc_status (BYTE pdrv) noexcept;
DRESULT disk_usbhmsc_read (BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) noexcept;
DRESULT disk_usbhmsc_write (BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) noexcept;
DRESULT disk_usbhmsc_ioctl (BYTE pdrv, BYTE cmd, void* buff) noexcept;


#ifdef __cplusplus
}
#endif

#endif