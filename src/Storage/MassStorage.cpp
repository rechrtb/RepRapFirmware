#include "MassStorage.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <ObjectModel/ObjectModel.h>

#if HAS_MASS_STORAGE

// Check that the LFN configuration in FatFS is sufficient
static_assert(FF_MAX_LFN >= MaxFilenameLength, "FF_MAX_LFN too small");

#endif

#if HAS_SBC_INTERFACE
# include <SBC/SbcInterface.h>
#endif

#ifdef DUET3_MB6HC
# include <GCodes/GCodeBuffer/GCodeBuffer.h>
#endif

#include "SdCard.h"
#include "UsbFlashDrive.h"

// A note on using mutexes:
// Each SD card volume has its own mutex. There is also one for the file table, and one for the find first/find next buffer.
// The FatFS subsystem locks and releases the appropriate volume mutex when it is called.
// Any function that needs to acquire both the file table mutex and a volume mutex MUST take the file table mutex first, to avoid deadlocks.
// Any function that needs to acquire both the find buffer mutex and a volume mutex MUST take the find buffer mutex first, to avoid deadlocks.
// No function should need to take both the file table mutex and the find buffer mutex.
// No function in here should be called when the caller already owns the shared SPI mutex.

#if HAS_MASS_STORAGE

// Private data and methods

# if SAME70
alignas(4) static __nocache char writeBufferStorage[NumFileWriteBuffers][FileWriteBufLen];
# endif

static SdCard sdCards[NumSdCards] = { SdCard("SDO", 0),  SdCard("SD1", 1) };
static UsbFlashDrive usbDrives[NumUsbDrives] = {UsbFlashDrive("USB0", 2)};

static StorageDevice* storageDevices[] = {&sdCards[0], &sdCards[1], &usbDrives[0]};

static DIR findDir;
#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
static Mutex dirMutex;
static FileInfoParser infoParser;
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
static FileWriteBuffer *freeWriteBuffers;
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
static Mutex fsMutex;
static FileStore files[MAX_FILES];
#endif

// Construct a full path name from a path and a filename. Returns false if error i.e. filename too long
/*static*/ bool MassStorage::CombineName(const StringRef& outbuf, const char* directory, const char* fileName) noexcept
{
	bool hadError = false;
	if (directory != nullptr && directory[0] != 0 && fileName[0] != '/' && (strlen(fileName) < 2 || !isdigit(fileName[0]) || fileName[1] != ':'))
	{
		hadError = outbuf.copy(directory);
		if (!hadError)
		{
			const size_t len = outbuf.strlen();
			if (len != 0 && outbuf[len - 1] != '/')
			{
				hadError = outbuf.cat('/');
			}
		}
	}
	else
	{
		outbuf.Clear();
	}
	if (!hadError)
	{
		hadError = outbuf.cat(fileName);
	}
	if (hadError)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Filename too long: cap=%u, dir=%.12s%s name=%.12s%s\n",
										outbuf.Capacity(),
										directory, (strlen(directory) > 12 ? "..." : ""),
										fileName, (strlen(fileName) > 12 ? "..." : "")
									 );
		outbuf.copy("?????");
	}
	return !hadError;
}

#if HAS_EMBEDDED_FILES && defined(DUET3_MB6HC)
size_t MassStorage::GetNumVolumes() noexcept { return 1; }
#endif

#if HAS_MASS_STORAGE

# ifdef DUET3_MB6HC

// Return the number of volumes, which on the 6HC is normally 1 but can be increased to 2
size_t MassStorage::GetNumVolumes() noexcept
{
	size_t count = 0;
	for (StorageDevice* device : storageDevices)
	{
		count += device->Useable();
	}
	return count;
}

// Configure additional SD card slots
// The card detect pin may be NoPin if the SD card slot doesn't support card detect
GCodeResult MassStorage::ConfigureSdCard(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	int slot = gb.GetLimitedUIValue('D', 1, 2);		// only slot 1 may be configured
	return sdCards[slot].SetCSPin(gb, reply);
}

# endif


// Sequence number management
uint16_t MassStorage::GetVolumeSeq(unsigned int volume) noexcept
{
	return storageDevices[volume]->GetSequenceNum();
}

// If 'path' is not the name of a temporary file, update the sequence number of its volume
// Return true if we did update the sequence number
static bool VolumeUpdated(const char *path) noexcept
{
	if (!StringEndsWithIgnoreCase(path, ".part")
#if HAS_SBC_INTERFACE
		&& !reprap.UsingSbcInterface()
#endif
	   )
	{
		const unsigned int volume = (isdigit(path[0]) && path[1] == ':') ? path[0] - '0' : 0;
		if (volume < ARRAY_SIZE(storageDevices))
		{
			storageDevices[volume]->IncrementSeq();
			return true;
		}
	}
	return false;
}

static time_t ConvertTimeStamp(uint16_t fdate, uint16_t ftime) noexcept
{
	struct tm timeInfo;
	memset(&timeInfo, 0, sizeof(timeInfo));
	timeInfo.tm_year = (fdate >> 9) + 80;
	const uint16_t month = (fdate >> 5) & 0x0F;
	timeInfo.tm_mon = (month == 0) ? month : month - 1;		// month is 1..12 in FAT but 0..11 in struct tm
	timeInfo.tm_mday = max<int>(fdate & 0x1F, 1);
	timeInfo.tm_hour = (ftime >> 11) & 0x1F;
	timeInfo.tm_min = (ftime >> 5) & 0x3F;
	timeInfo.tm_sec = (ftime & 0x1F) * 2;
	timeInfo.tm_isdst = 0;
	return mktime(&timeInfo);
}
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

void MassStorage::Init() noexcept
{
	fsMutex.Create("FileSystem");

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES
	dirMutex.Create("DirSearch");
#endif

# if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	freeWriteBuffers = nullptr;
	for (size_t i = 0; i < NumFileWriteBuffers; ++i)
	{
#  if SAME70
		freeWriteBuffers = new FileWriteBuffer(freeWriteBuffers, writeBufferStorage[i]);
#  else
		freeWriteBuffers = new FileWriteBuffer(freeWriteBuffers);
#  endif
	}
# endif

# if HAS_MASS_STORAGE
	for (StorageDevice* device : storageDevices)
	{
		device->Init();
	}
	// We no longer mount the SD card here because it may take a long time if it fails
# endif
}


#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES
Mutex& GetFsMutex()
{
	return fsMutex;
}
#endif

void MassStorage::Spin() noexcept
{
# if HAS_MASS_STORAGE
	for (StorageDevice* device : storageDevices)
	{
		device->Init();
	}
# endif

	// Check if any files are supposed to be closed
	{
		MutexLocker lock(fsMutex);
		for (FileStore & fil : files)
		{
			if (fil.IsCloseRequested())
			{
				// We could not close this file in an ISR, so do it here
				fil.Close();
			}
		}
	}
}

FileStore* MassStorage::OpenFile(const char* filePath, OpenMode mode, uint32_t preAllocSize) noexcept
{
	{
		MutexLocker lock(fsMutex);
		for (FileStore& fs : files)
		{
			if (fs.IsFree())
			{
				FileStore * const ret = (fs.Open(filePath, mode, preAllocSize)) ? &fs: nullptr;
# if HAS_MASS_STORAGE
				if (ret != nullptr && (mode == OpenMode::write || mode == OpenMode::writeWithCrc))
				{
					(void)VolumeUpdated(filePath);
				}
# endif
				return ret;
			}
		}
	}
	reprap.GetPlatform().Message(ErrorMessage, "Max open file count exceeded.\n");
	return nullptr;
}

# if (HAS_MASS_STORAGE || HAS_EMBEDDED_FILES) && SUPPORT_ASYNC_MOVES

// Duplicate a file handle, with the duplicate having its own position in the file. Use only with files opened in read-only mode.
FileStore *MassStorage::DuplicateOpenHandle(const FileStore *f) noexcept
{
	if (f == nullptr || !f->IsOpen())
	{
		return nullptr;
	}

	MutexLocker lock(fsMutex);
	for (FileStore& fs : files)
	{
		if (fs.IsFree())
		{
			fs.CopyFrom(f);
			return &fs;
		}
	}
	reprap.GetPlatform().Message(ErrorMessage, "Max open file count exceeded.\n");
	return nullptr;
}

# endif

// Close all files. Called only from Platform::Exit.
void MassStorage::CloseAllFiles() noexcept
{
	MutexLocker lock(fsMutex);
	for (FileStore& f : files)
	{
		while (!f.IsFree())			// a file may have a use count greater then one, so need to loop here
		{
			f.Close();
		}
	}
}

#endif	// HAS_MASS_STORAGE || HAS_SBC_INTERFACE || HAS_EMBEDDED_FILES

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

// Check if the specified directory exists
bool MassStorage::DirectoryExists(const char *path) noexcept
{
	// Remove any trailing '/' from the directory name, it sometimes (but not always) confuses f_opendir
	String<MaxFilenameLength> loc;
	loc.copy(path);
	return DirectoryExists(loc.GetRef());
}

// Check if the specified directory exists
// Warning: if 'path' has a trailing '/' or '\\' character, it will be removed!
bool MassStorage::DirectoryExists(const StringRef& path) noexcept
{
	// Remove any trailing '/' from the directory name, it sometimes (but not always) confuses f_opendir
	const size_t len = path.strlen();
	if (len != 0 && (path[len - 1] == '/' || path[len - 1] == '\\'))
	{
		path.Truncate(len - 1);
	}

# if HAS_MASS_STORAGE
	DIR dir;
	const bool ok = (f_opendir(&dir, path.c_str()) == FR_OK);
	if (ok)
	{
		f_closedir(&dir);
	}
	return ok;
# elif HAS_EMBEDDED_FILES
	return EmbeddedFiles::DirectoryExists(path);
# endif
}

#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Static helper functions
size_t FileWriteBuffer::fileWriteBufLen = FileWriteBufLen;

FileWriteBuffer *MassStorage::AllocateWriteBuffer() noexcept
{
	MutexLocker lock(fsMutex);

	FileWriteBuffer * const buffer = freeWriteBuffers;
	if (buffer != nullptr)
	{
		freeWriteBuffers = buffer->Next();
		buffer->SetNext(nullptr);
		buffer->DataTaken();				// make sure that the write pointer is clear
	}
	return buffer;
}

void MassStorage::ReleaseWriteBuffer(FileWriteBuffer *buffer) noexcept
{
	MutexLocker lock(fsMutex);
	buffer->SetNext(freeWriteBuffers);
	freeWriteBuffers = buffer;
}

# if HAS_SBC_INTERFACE

// Return true if any files are open on the file system
bool MassStorage::AnyFileOpen() noexcept
{
	MutexLocker lock(fsMutex);
	for (const FileStore & fil : files)
	{
		if (fil.IsOpen())
		{
			return true;
		}
	}
	return false;
}

// Invalidate all files
void MassStorage::InvalidateAllFiles() noexcept
{
	MutexLocker lock(fsMutex);
	for (FileStore& f : files)
	{
		if (f.IsOpen())
		{
			f.Invalidate();
		}
	}
}

# endif

# if HAS_MASS_STORAGE

// Delete a file or directory
static bool InternalDelete(const char* filePath, ErrorMessageMode errorMessageMode) noexcept
{
	FRESULT unlinkReturn;
	bool isOpen = false;

	// Start new scope to lock the filesystem for the minimum time
	{
		MutexLocker lock(fsMutex);

		// First check whether the file is open - don't allow it to be deleted if it is, because that may corrupt the file system
		FIL file;
		const FRESULT openReturn = f_open(&file, filePath, FA_OPEN_EXISTING | FA_READ);
		if (openReturn == FR_OK)
		{
			for (const FileStore& fil : files)
			{
				if (fil.IsSameFile(file))
				{
					isOpen = true;
					break;
				}
			}
			f_close(&file);
		}

		if (!isOpen)
		{
			unlinkReturn = f_unlink(filePath);
		}
	}

	if (isOpen)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Cannot delete file %s because it is open\n", filePath);
		return false;
	}

	if (unlinkReturn != FR_OK)
	{
		// If the error was that the file or path doesn't exist, don't generate a global error message, but still return false
		if (   errorMessageMode == ErrorMessageMode::messageAlways
			|| (errorMessageMode == ErrorMessageMode::messageUnlessMissing && unlinkReturn != FR_NO_FILE && unlinkReturn != FR_NO_PATH)
		   )
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to delete %s%s\n",
											filePath,
											(unlinkReturn == FR_NOT_EMPTY) ? " because it is a folder and not empty" : ""
										 );
		}
		return false;
	}
	return true;
}

// Delete the contents of an open directory returning true if successful
// File system must be locked before calling this
// This is recursive. In order to avoid using large amounts of stack it uses the string referred to by filePath to hold the name of each contained file as it is deleted.
static bool DeleteContents(DIR& dir, const StringRef& filePath, ErrorMessageMode errorMessageMode) noexcept
{
	const size_t originalPathLength = filePath.strlen();
	size_t pathLength = originalPathLength;
	if (originalPathLength == 0 || filePath[originalPathLength - 1] != '/')
	{
		filePath.cat('/');
		++pathLength;
	}

	bool ok = true;
	while (ok)
	{
		FILINFO entry;
		const FRESULT res = f_readdir(&dir, &entry);
		if (res != FR_OK || entry.fname[0] == 0)
		{
			break;
		}
		if (!StringEqualsIgnoreCase(entry.fname, ".") && !StringEqualsIgnoreCase(entry.fname, ".."))
		{
			filePath.cat(entry.fname);
			if (entry.fattrib & AM_DIR)
			{
				DIR dir2;
				if (f_opendir(&dir2, filePath.c_str()) == FR_OK)
				{
					const bool ok = DeleteContents(dir, filePath, errorMessageMode);
					f_closedir(&dir2);
					if (!ok)
					{
						return false;
					}
				}
				else
				{
					ok = false;
				}
			}
			else
			{
				if (!InternalDelete(filePath.c_str(), errorMessageMode))
				{
					ok = false;
				}
			}
			filePath.Truncate(pathLength);
		}
	}

	filePath.Truncate(originalPathLength);
	return true;
}

# endif

// Delete a file or directory and update the volume sequence number returning true if successful
// Note, we use the filePath string to build up nested directories if we are doing a recursive delete
bool MassStorage::Delete(const StringRef& filePath, ErrorMessageMode errorMessageMode, bool recursive) noexcept
{
# if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		if (reprap.GetSbcInterface().DeleteFileOrDirectory(filePath.c_str(), recursive))
		{
			return true;
		}

		if (errorMessageMode != ErrorMessageMode::noMessage)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to delete file %s\n", filePath.c_str());
		}
		return false;
	}
# endif

# if HAS_MASS_STORAGE
	if (recursive)
	{
		// Check for trying to delete root
		const char *fp = filePath.c_str();
		if (isDigit(*fp) && fp[1] == ':')
		{
			fp += 2;
		}
		while (*fp == '/')
		{
			++fp;
		}
		if (*fp == 0)
		{
			if (errorMessageMode != ErrorMessageMode::noMessage)
			{
				reprap.GetPlatform().Message(ErrorMessage, "Delete root folder is not allowed");
			}
			return false;
		}

		MutexLocker locker(fsMutex);
		DIR dir;
		if (f_opendir(&dir, filePath.c_str()) == FR_OK)
		{
			const bool ok = DeleteContents(dir, filePath, errorMessageMode);
			f_closedir(&dir);
			if (!ok)
			{
				(void)VolumeUpdated(filePath.c_str());			// in case we deleted any contained files
				return false;
			}
		}
	}

	const bool ok = InternalDelete(filePath.c_str(), errorMessageMode);
	if (ok)
	{
		(void)VolumeUpdated(filePath.c_str());
	}
	return ok;
# else
	return false;
# endif
}

#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

// Open a directory to read a file list. Returns true if it contains any files, false otherwise.
// If this returns true then the file system mutex is owned. The caller must subsequently release the mutex either
// by calling FindNext until it returns false, or by calling AbandonFindNext.
bool MassStorage::FindFirst(const char *directory, FileInfo &file_info) noexcept
{
	// Remove any trailing '/' from the directory name, it sometimes (but not always) confuses f_opendir
	String<MaxFilenameLength> loc;
	loc.copy(directory);
	const size_t len = loc.strlen();
	if (len != 0 && (loc[len - 1] == '/' || loc[len - 1] == '\\'))
	{
		loc.Truncate(len - 1);
	}

	if (!dirMutex.Take(10000))
	{
		return false;
	}

#if HAS_MASS_STORAGE
	if (f_opendir(&findDir, loc.c_str()) == FR_OK)
	{
		FILINFO entry;
		for (;;)
		{
			const FRESULT res = f_readdir(&findDir, &entry);
			if (res != FR_OK || entry.fname[0] == 0) break;
			if (!StringEqualsIgnoreCase(entry.fname, ".") && !StringEqualsIgnoreCase(entry.fname, ".."))
			{
				file_info.isDirectory = (entry.fattrib & AM_DIR);
				file_info.fileName.copy(entry.fname);
				file_info.size = entry.fsize;
				file_info.lastModified = ConvertTimeStamp(entry.fdate, entry.ftime);
				return true;
			}
		}
		f_closedir(&findDir);
	}
#elif HAS_EMBEDDED_FILES
	if (EmbeddedFiles::FindFirst(directory, file_info))
	{
		return true;
	}
#endif

	dirMutex.Release();
	return false;
}

// Find the next file in a directory. Returns true if another file has been read.
// If it returns false then it also releases the mutex.
bool MassStorage::FindNext(FileInfo &file_info) noexcept
{
	if (dirMutex.GetHolder() != RTOSIface::GetCurrentTask())
	{
		return false;		// error, we don't hold the mutex
	}

#if HAS_MASS_STORAGE
	FILINFO entry;

	if (f_readdir(&findDir, &entry) == FR_OK && entry.fname[0] != 0)
	{
		file_info.isDirectory = (entry.fattrib & AM_DIR);
		file_info.size = entry.fsize;
		file_info.fileName.copy(entry.fname);
		file_info.lastModified = ConvertTimeStamp(entry.fdate, entry.ftime);
		return true;
	}

	f_closedir(&findDir);
#elif HAS_EMBEDDED_FILES
	if (EmbeddedFiles::FindNext(file_info))
	{
		return true;
	}
#endif
	// TODO implement SBC interface for this

	dirMutex.Release();
	return false;
}

// Quit searching for files. Needed to avoid hanging on to the mutex. Safe to call even if the caller doesn't hold the mutex.
void MassStorage::AbandonFindNext() noexcept
{
	if (dirMutex.GetHolder() == RTOSIface::GetCurrentTask())
	{
		dirMutex.Release();
	}
}

#endif

#if HAS_MASS_STORAGE

// Month names. The first entry is used for invalid month numbers.
static const char *monthNames[13] = { "???", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };

// Returns the name of the specified month or '???' if the specified value is invalid.
const char* MassStorage::GetMonthName(const uint8_t month) noexcept
{
	return (month <= 12) ? monthNames[month] : monthNames[0];
}

// Ensure that the path up to the last '/' (excluding trailing '/' characters) in filePath exists, returning true if successful
bool MassStorage::EnsurePath(const char* filePath, bool messageIfFailed) noexcept
{
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		// This isn't needed in SBC mode because DSF does it automatically
		return true;
	}
#endif

	// Try to create the path of this file if we want to write to it
	String<MaxFilenameLength> filePathCopy;
	filePathCopy.copy(filePath);

	size_t i = (isdigit(filePathCopy[0]) && filePathCopy[1] == ':') ? 2 : 0;
	if (filePathCopy[i] == '/')
	{
		++i;
	}

	size_t limit = filePathCopy.strlen();
	while (limit != 0 && filePath[limit - 1] == '/')
	{
		--limit;
	}
	while (i < limit)
	{
		if (filePathCopy[i] == '/')
		{
			filePathCopy[i] = 0;
			if (!MassStorage::DirectoryExists(filePathCopy.GetRef()) && f_mkdir(filePathCopy.c_str()) != FR_OK)
			{
				if (messageIfFailed)
				{
					reprap.GetPlatform().MessageF(ErrorMessage, "Failed to create folder %s in path %s\n", filePathCopy.c_str(), filePath);
				}
				return false;
			}
			filePathCopy[i] = '/';
		}
		++i;
	}
	return true;
}

// Create a new directory
bool MassStorage::MakeDirectory(const char *directory, bool messageIfFailed) noexcept
{
	if (!EnsurePath(directory, messageIfFailed))
	{
		return false;
	}
	if (f_mkdir(directory) != FR_OK)
	{
		if (messageIfFailed)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to create folder %s\n", directory);
		}
		return false;
	}
	(void)VolumeUpdated(directory);
	return true;
}

// Rename a file or directory, optionally deleting the existing one if it exists
bool MassStorage::Rename(const char *oldFilename, const char *newFilename, bool deleteExisting, bool messageIfFailed) noexcept
{
	// Check the the old file exists before we possibly delete any existing file with the new name
	if (!FileExists(oldFilename) && !DirectoryExists(oldFilename))
	{
		if (messageIfFailed)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to rename file %s: file not found\n", oldFilename);
		}
		return false;
	}
	if (newFilename[0] >= '0' && newFilename[0] <= '9' && newFilename[1] == ':')
	{
		// Workaround for DWC 1.13 which sends a volume specification at the start of the new path.
		// f_rename can't handle this, so skip past the volume specification.
		// We are assuming that the user isn't really trying to rename across volumes. This is a safe assumption when the client is DWC.
		newFilename += 2;
	}
	if (!EnsurePath(newFilename, messageIfFailed))
	{
		return false;
	}
	if (deleteExisting && (FileExists(newFilename) || DirectoryExists(newFilename)))
	{
		if (!InternalDelete(newFilename, ErrorMessageMode::messageAlways))
		{
			return false;
		}
	}
	if (f_rename(oldFilename, newFilename) != FR_OK)
	{
		if (messageIfFailed)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "Failed to rename file or directory %s to %s\n", oldFilename, newFilename);
		}
		return false;
	}

	if (!VolumeUpdated(oldFilename))				// only update the sequence number once
	{
		(void)VolumeUpdated(newFilename);
	}
	return true;
}
#endif

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Check if the specified file exists
bool MassStorage::FileExists(const char *filePath) noexcept
{
#if HAS_SBC_INTERFACE
	if (reprap.UsingSbcInterface())
	{
		return reprap.GetSbcInterface().FileExists(filePath);
	}
#endif

#if HAS_MASS_STORAGE
	FILINFO fil;
	return (f_stat(filePath, &fil) == FR_OK);
#else
	return false;
#endif
}

#endif

#if HAS_MASS_STORAGE

// Return the last modified time of a file, or zero if failure
time_t MassStorage::GetLastModifiedTime(const char *filePath) noexcept
{
	FILINFO fil;
	if (f_stat(filePath, &fil) == FR_OK)
	{
		return ConvertTimeStamp(fil.fdate, fil.ftime);
	}
	return 0;
}

bool MassStorage::SetLastModifiedTime(const char *filePath, time_t time) noexcept
{
	tm timeInfo;
	gmtime_r(&time, &timeInfo);
	FILINFO fno;
    fno.fdate = (WORD)(((timeInfo.tm_year - 80) * 512U) | (timeInfo.tm_mon + 1) * 32U | timeInfo.tm_mday);
    fno.ftime = (WORD)(timeInfo.tm_hour * 2048U | timeInfo.tm_min * 32U | timeInfo.tm_sec / 2U);
    const bool ok = (f_utime(filePath, &fno) == FR_OK);
    if (!ok)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to set last modified time for file '%s'\n", filePath);
	}
    return ok;
}

// Check if the drive referenced in the specified path is mounted. Return true if it is.
// Ideally we would try to mount it if it is not, however mounting a drive can take a long time, and the functions that call this are expected to execute quickly.
bool MassStorage::CheckDriveMounted(const char* path) noexcept
{
	const size_t device = (strlen(path) >= 2 && path[1] == ':' && isDigit(path[0]))
						? path[0] - '0'
						: 0;
	return device < GetNumVolumes() && storageDevices[device]->IsMounted();
}

// Return true if any files are open on the file system
bool MassStorage::AnyFileOpen(const FATFS *fs) noexcept
{
	MutexLocker lock(fsMutex);
	for (const FileStore & fil : files)
	{
		if (fil.IsOpenOn(fs))
		{
			return true;
		}
	}
	return false;
}

// Invalidate all open files on the specified file system, returning the number of files that were invalidated
unsigned int MassStorage::InvalidateFiles(const FATFS *fs) noexcept
{
	unsigned int invalidated = 0;
	MutexLocker lock(fsMutex);
	for (FileStore & fil : files)
	{
		if (fil.Invalidate(fs))
		{
			++invalidated;
		}
	}
	return invalidated;
}

bool MassStorage::IsCardDetected(size_t card) noexcept
{
	return storageDevices[card]->IsPresent();
}

#endif

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

// Mount the specified SD card, returning true if done, false if needs to be called again.
// If an error occurs, return true with the error message in 'reply'.
// This may only be called to mount one card at a time.
GCodeResult MassStorage::Mount(size_t card, const StringRef& reply, bool reportSuccess) noexcept
{
	if (card >= GetNumVolumes())
	{
		reply.copy("SD card number out of range");
		return GCodeResult::error;
	}

	return storageDevices[card]->Mount(card, reply, reportSuccess);
}

// Unmount the specified SD card, returning true if done, false if needs to be called again.
// If an error occurs, return true with the error message in 'reply'.
GCodeResult MassStorage::Unmount(size_t card, const StringRef& reply) noexcept
{
	if (card >= GetNumVolumes())
	{
		reply.copy("SD card number out of range");
		return GCodeResult::error;
	}

# if HAS_MASS_STORAGE
	if (AnyFileOpen(storageDevices[card]->GetFS()))
	{
		// Don't unmount the card if any files are open on it
		reply.copy("SD card has open file(s)");
		return GCodeResult::error;
	}

	storageDevices[card]->Unmount();
	reply.printf("SD card %u may now be removed", card);
	storageDevices[card]->IncrementSeq();
# endif

	return GCodeResult::ok;
}

bool MassStorage::IsDriveMounted(size_t drive) noexcept
{
	return drive < GetNumVolumes()
#if HAS_MASS_STORAGE
		&& storageDevices[drive]->IsMounted()
#endif
		;
}

unsigned int MassStorage::GetNumFreeFiles() noexcept
{
	unsigned int numFreeFiles = 0;
	MutexLocker lock(fsMutex);
	for (const FileStore & fil : files)
	{
		if (fil.IsFree())
		{
			++numFreeFiles;
		}
	}
	return numFreeFiles;
}

GCodeResult MassStorage::GetFileInfo(const char *filePath, GCodeFileInfo& info, bool quitEarly) noexcept
{
	return infoParser.GetFileInfo(filePath, info, quitEarly);
}

void MassStorage::Diagnostics(MessageType mtype) noexcept
{
	Platform& platform = reprap.GetPlatform();

	// Show the number of free entries in the file table
	platform.MessageF(mtype, "=== Storage ===\nFree file entries: %u\n", MassStorage::GetNumFreeFiles());

# if HAS_MASS_STORAGE
	SdCard &sd0 = sdCards[0];
#  if HAS_HIGH_SPEED_SD
	// Show the HSMCI CD pin and speed
	platform.MessageF(mtype, "SD card 0 %s, interface speed: %.1fMBytes/sec\n",
								(sd0.IsPresent() ? "detected" : "not detected"), static_cast<double>(sd0.GetInterfaceSpeed() * 0.000001f));
#  else
	platform.MessageF(mtype, "SD card 0 %s\n", (sd0.IsPresent() ? "detected" : "not detected"));
#  endif

	SdCard::Stats stats;
	sd0.GetStats(stats);

	// Show the longest SD card write time
	platform.MessageF(mtype, "SD card 0 longest read time %.1fms, write time %.1fms, max retries %u\n",
								(double)stats.maxReadTime, (double)stats.maxWriteTime, static_cast<unsigned int>(stats.maxRetryCount));

	sd0.ResetStats();
# endif
}

#endif

#if HAS_MASS_STORAGE

// Append the simulated printing time to the end of the file
void MassStorage::RecordSimulationTime(const char *printingFilePath, uint32_t simSeconds) noexcept
{
	FileStore * const file = OpenFile(printingFilePath, OpenMode::append, 0);
	bool ok = (file != nullptr);
	if (ok)
	{
		// Check whether there is already simulation info at the end of the file, in which case we should replace it
		constexpr size_t BufferSize = 100;
		String<BufferSize> buffer;
		const size_t bytesToRead = (size_t)min<FilePosition>(file->Length(), BufferSize);
		const FilePosition seekPos = file->Length() - bytesToRead;
		ok = file->Seek(seekPos);
		time_t lastModtime = 0;
		if (ok)
		{
			ok = (file->Read(buffer.GetRef().Pointer(), bytesToRead) == (int)bytesToRead);
			if (ok)
			{
				lastModtime = GetLastModifiedTime(printingFilePath);			// save the last modified time to that we can restore it later
				buffer[bytesToRead] = 0;										// this is OK because String<N> has N+1 bytes of storage
				const char* const pos = strstr(buffer.c_str(), FileInfoParser::SimulatedTimeString);
				if (pos != nullptr)
				{
					ok = file->Seek(seekPos + (pos - buffer.c_str()));			// overwrite previous simulation time
				}
				if (ok)
				{
					buffer.printf("%s: %" PRIu32 "\n", FileInfoParser::SimulatedTimeString, simSeconds);
					ok = file->Write(buffer.c_str());
					if (ok)
					{
						ok = file->Truncate();									// truncate file in case we overwrote a previous longer simulation time
					}
				}
			}
		}
		if (!file->Close())
		{
			ok = false;
		}
		if (ok && lastModtime != 0)
		{
			ok = SetLastModifiedTime(printingFilePath, lastModtime);
		}
	}

	if (!ok)
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "Failed to append simulated print time to file %s\n", printingFilePath);
	}
}

// Get information about the SD card and interface speed
MassStorage::InfoResult MassStorage::GetCardInfo(size_t slot, SdCardReturnedInfo& returnedInfo) noexcept
{
	if (slot >= GetNumVolumes())
	{
		return InfoResult::badSlot;
	}

	StorageDevice* card = storageDevices[slot];

	if (!card->IsMounted())
	{
		return InfoResult::noCard;
	}

	returnedInfo.cardCapacity = card->GetCapacity();
	returnedInfo.partitionSize = card->GetPartitionSize();
	returnedInfo.freeSpace = card->GetFreeSpace();
	returnedInfo.clSize = card->GetClusterSize();
	returnedInfo.speed = card->GetInterfaceSpeed();

	return InfoResult::ok;
}

Mutex& MassStorage::GetFsMutex() noexcept
{
	return fsMutex;
}

# if SUPPORT_OBJECT_MODEL

const ObjectModel * MassStorage::GetVolume(size_t vol) noexcept
{
	return storageDevices[vol];
}

# endif
#endif

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
		//storageDevices[vol].GetMutex().Take();
		return 1;
	}

	// Unlock sync object
	void ff_mutex_give (int vol) noexcept
	{
		//storageDevices[vol].GetMutex().Release();
	}

	// Delete a sync object
	void ff_mutex_delete (int vol) noexcept
	{
		// nothing to do, we never delete the mutex
	}
}


// End
