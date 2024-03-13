#include "ff.h"				// for type definitions
#include "diskio.h"
#include "diskio_usbhmsc.h"
#include "tusb.h"

static volatile bool _disk_busy[CFG_TUH_DEVICE_MAX];

static bool disk_io_complete(uint8_t dev_addr, tuh_msc_complete_data_t const * cb_data)
{
  (void) dev_addr; (void) cb_data;
  _disk_busy[dev_addr-1] = false;
  return true;
}

static void wait_for_disk_io(BYTE pdrv)
{
  while(_disk_busy[pdrv])
  {
    delay(20);
  }
}


DSTATUS disk_usbhmsc_initialize (BYTE pdrv) noexcept
{
  (void) pdrv;
	return 0; // nothing to do
}

DSTATUS disk_usbhmsc_status (BYTE pdrv) noexcept
{
  uint8_t dev_addr = pdrv + 1;
  return tuh_msc_mounted(dev_addr) ? 0 : STA_NODISK;
}

DRESULT disk_usbhmsc_read (BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) noexcept
{
	uint8_t const dev_addr = pdrv + 1;
	uint8_t const lun = 0;

	_disk_busy[pdrv] = true;
	tuh_msc_read10(dev_addr, lun, buff, sector, (uint16_t) count, disk_io_complete, 0);
	wait_for_disk_io(pdrv);

	return RES_OK;
}

DRESULT disk_usbhmsc_write (BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) noexcept
{
	uint8_t const dev_addr = pdrv + 1;
	uint8_t const lun = 0;

	_disk_busy[pdrv] = true;
	tuh_msc_write10(dev_addr, lun, buff, sector, (uint16_t) count, disk_io_complete, 0);
	wait_for_disk_io(pdrv);
    return RES_OK;
}

DRESULT disk_usbhmsc_ioctl (BYTE pdrv, BYTE cmd, void* buff) noexcept
{
  uint8_t const dev_addr = pdrv + 1;
  uint8_t const lun = 0;
  switch ( cmd )
  {
    case CTRL_SYNC:
      // nothing to do since we do blocking
      return RES_OK;

    case GET_SECTOR_COUNT:
      *((DWORD*) buff) = (WORD) tuh_msc_get_block_count(dev_addr, lun);
      return RES_OK;

    case GET_SECTOR_SIZE:
      *((WORD*) buff) = (WORD) tuh_msc_get_block_size(dev_addr, lun);
      return RES_OK;

    case GET_BLOCK_SIZE:
      *((DWORD*) buff) = 1;    // erase block size in units of sector size
      return RES_OK;

    default:
      return RES_PARERR;
  }

	return RES_OK;
}