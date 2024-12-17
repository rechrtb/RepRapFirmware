/*
 * FileInfoParser.cpp
 *
 *  Created on: 31 Mar 2018
 *      Author: David
 */

#include "FileInfoParser.h"

#if HAS_MASS_STORAGE || HAS_EMBEDDED_FILES

#include <Platform/OutputMemory.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <PrintMonitor/PrintMonitor.h>
#include <GCodes/GCodes.h>
#include <ObjectModel/GlobalVariables.h>
#include <GCodes/GCodeBuffer/ExpressionParser.h>

constexpr FileInfoParser::ParseTableEntry FileInfoParser::parseTable[] =
{
	// Note: if a key string in this table is a leading or embedded substring of another, the longer one must come first
	// First character is case-insensitive but must be uppercase in this table, remaining characters are case-sensitive
	{	"Build time",								&FileInfoParser::ProcessJobTime,			0 },		// S3D								";   Build time: 0 hours 42 minutes"
																											// also REALvision					"; Build time: 2:11:47"
	{	"Calculated-during-export Build Time",		&FileInfoParser::ProcessJobTime,			0 },		// KISSSlicer 2 alpha				"; Calculated-during-export Build Time: 130.62 minutes"
	{	"CustomInfo",								&FileInfoParser::ProcessCustomInfo,			0 },		// OEMs can include this in their GCode files
	{	"Estimated Build Time",						&FileInfoParser::ProcessJobTime,			0 },		// KISSlicer						"; Estimated Build Time:   332.83 minutes"
	{	"Estimated Build Volume",					&FileInfoParser::ProcessFilamentUsed,		3 },		// Kisslicer older versions filament volume
	{	"Estimated printing time (normal mode)",	&FileInfoParser::ProcessJobTime,			0 },		// PrusaSlicer later versions		"; estimated printing time (normal mode) = 2d 1h 5m 24s"
	{	"Estimated printing time",					&FileInfoParser::ProcessJobTime,			0 },		// PrusaSlicer older versions		"; estimated printing time = 1h 5m 24s"
	{	"Extruder",									&FileInfoParser::ProcessFilamentUsed,		5 },		// Fusion 360 						";Extruder 1 material used: 1811mm"
	{	"Ext",										&FileInfoParser::ProcessFilamentUsed,		2 },		// Kisslicer newer versions filament by extruder
	{	"Filament length",							&FileInfoParser::ProcessFilamentUsed,		1 },		// S3D v4							";   Filament length: 13572.2 mm (13.57 m)"
	{	"Filament used [mm]",						&FileInfoParser::ProcessFilamentUsed,		0 },		// Prusa slicer, OrcaSlicer			"; filament used [mm] = 1965.97"
	{	"Filament used",							&FileInfoParser::ProcessFilamentUsed,		0 },		// Cura								";Filament used: 0m"
																											// also Kiri Moto					"; --- filament used: 1657.31 mm ---"
	{	"Fusion version",							&FileInfoParser::ProcessGeneratedBy,		2 },		// Fusion 360 generated-by
	{	"G-Code generated by",						&FileInfoParser::ProcessGeneratedBy,		0 },		// S3D generated-by
	{	"GENERATOR.NAME",							&FileInfoParser::ProcessGeneratedBy,		0 },		// Pathio  generated-by(the version is separate, we don't include that)
	{	"Generated by",								&FileInfoParser::ProcessGeneratedBy,		0 },		// kiri:moto generated-by
																											// also slic3r and PrusaSlicer generated-by
	{	"Generated with",							&FileInfoParser::ProcessGeneratedBy,		0 },		// Cura (new) generated-by
																											// also Matter Control generated-by
	{	"Height",									&FileInfoParser::ProcessObjectHeight,		0 },		// Fusion 3							";Height: 250mm"
	{	"KISSlicer",								&FileInfoParser::ProcessGeneratedBy,		2 },		// KISSlicer generated-by
	{	"Layer count",								&FileInfoParser::ProcessNumLayers,			0 },		// Fusion 3 layer count				";Layer count: 125"
	{	"Layer height",								&FileInfoParser::ProcessLayerHeight,		0 },		// Cura
	{	"LayerHeight",								&FileInfoParser::ProcessLayerHeight,		0 },		// S3D								";   layerHeight,0.2"
	{	"LayerThickness",							&FileInfoParser::ProcessLayerHeight,		0 },		// Matter Control
	{	"Layer_height",								&FileInfoParser::ProcessLayerHeight,		0 },		// slic3r, PrusaSlicer, OrcaSlicer	"; layer_height = 0.2"
	{	"Layer_thickness_mm",						&FileInfoParser::ProcessLayerHeight,		0 },		// Kisslicer
	{	"Material Length",							&FileInfoParser::ProcessFilamentUsed,		1 },		// S3D v5
	{	"Material#",								&FileInfoParser::ProcessFilamentUsed,		4 },		// Ideamaker						";Material#1 Used: 868.0"
	{	"Max_z_height",								&FileInfoParser::ProcessObjectHeight,		0 },		// OrcaSlicer						"; max_z_height: 20.00"
	{	"NUM_LAYERS",								&FileInfoParser::ProcessNumLayers,			0 },
	{	"Num_layers",								&FileInfoParser::ProcessNumLayers,			0 },
	{	"PRINT.TIME",								&FileInfoParser::ProcessJobTime,			0 },		// Patio
	{	"Print Time",								&FileInfoParser::ProcessJobTime,			0 },		// Ideamaker
	{ 	"Print time",								&FileInfoParser::ProcessJobTime,			0 },		// Fusion 360						";Print time: 40m:36s"
	{	"Simulated print time",						&FileInfoParser::ProcessSimulatedTime,		0 },		// appended to the file by RRF
	{	"SliceHeight",								&FileInfoParser::ProcessLayerHeight,		0 },		// kiri:moto
	{	"Sliced at",								&FileInfoParser::ProcessGeneratedBy,		1 },		// Cura (old) generated-by
	{	"Sliced by",								&FileInfoParser::ProcessGeneratedBy,		0 },		// ideaMaker generated-by
	{	"TIME",										&FileInfoParser::ProcessJobTime,			0 },		// Cura								";TIME:38846"
																											// also Kiri Moto					";TIME 3720.97"
	{	"Thumbnail begin",							&FileInfoParser::ProcessThumbnail,			0 },		// thumbnail in PNG format
	{	"Thumbnail_JPG begin",						&FileInfoParser::ProcessThumbnail,			2 },		// thumbnail in JPEG format
	{	"Thumbnail_QOI begin",						&FileInfoParser::ProcessThumbnail,			1 },		// thumbnail in QOI format
	{	"Total layer number",						&FileInfoParser::ProcessNumLayers,			0 },		// OrcaSlicer layer count			"; total layer number: 100"
	{	"Total print time (s)",						&FileInfoParser::ProcessJobTime,			0 },		// Matter Control print time
};

// Return true if the first string comes before the second, where a string comes before its leading substrings
constexpr bool ComesBefore(const char *_ecv_array s1, const char *_ecv_array s2) noexcept
{
	return *s1 != 0 && (*s2 == 0 || *s2 > *s1 || (*s2 == *s1 && ComesBefore(s1 + 1, s2 + 1)));
}

// Return true if the keys in the table are all uppercase letters and nn the correct order
constexpr bool FileInfoParser::TableIsCorrectlyOrdered() noexcept
{
	for (size_t i = 0; i + 1 < ARRAY_SIZE(FileInfoParser::parseTable); ++i)
	{
		if (parseTable[i].key[0] < 'A' || parseTable[i].key[0] > 'Z' || parseTable[i + 1].key[0] < 'A' || parseTable[i + 1].key[0] > 'Z')
		{
			return false;
		}
		if (!ComesBefore(FileInfoParser::parseTable[i].key, FileInfoParser::parseTable[i + 1].key))
		{
			return false;
		}
	}
	return true;
}

static_assert(FileInfoParser::TableIsCorrectlyOrdered());

FileInfoParser::FileInfoParser() noexcept
	:  fileBeingParsed(nullptr), parseState(FileParseState::notParsing), accumulatedParseTime(0), accumulatedReadTime(0), accumulatedSeekTime(0)
{
	parsedFileInfo.Init();
	parserMutex.Create("FileInfoParser");
}

// This following method needs to be called repeatedly until it returns true - this may take a few runs
GCodeResult FileInfoParser::GetFileInfo(const char *_ecv_array filePath, GCodeFileInfo& info, bool quitEarly, GlobalVariables *_ecv_null customVariables) noexcept
{
	MutexLocker lock(parserMutex, MaxFileinfoProcessTime);
	if (!lock.IsAcquired())
	{
		return GCodeResult::notFinished;
	}

	if (parseState != FileParseState::notParsing && !StringEqualsIgnoreCase(filePath, filenameBeingParsed.c_str()))
	{
		// We are already parsing a different file
		if (millis() - lastFileParseTime < MaxFileParseInterval)
		{
			return GCodeResult::notFinished;				// try again later
		}

		// Time this client out because it has probably disconnected
		fileBeingParsed->Close();
		parseState = FileParseState::notParsing;
	}

	if (parseState == FileParseState::notParsing)
	{
		if (reprap.Debug(Module::PrintMonitor))
		{
			reprap.GetPlatform().MessageF(UsbMessage, "Processing file %s\n", filePath);
		}
		// See if we can access the file
		// Webserver may call rr_fileinfo for a directory, check this case here
		const uint32_t now = millis();
		if (MassStorage::DirectoryExists(filePath))
		{
			info.isValid = false;
			return GCodeResult::ok;
		}

		fileBeingParsed = MassStorage::OpenFile(filePath, OpenMode::read, 0);
		if (fileBeingParsed == nullptr)
		{
			// Something went wrong - we cannot open it
			info.isValid = false;
			return GCodeResult::error;
		}

		// File has been opened, let's start now
		filenameBeingParsed.copy(filePath);

		// Set up the info struct
		parsedFileInfo.Init();
		parsedFileInfo.fileSize = fileBeingParsed->Length();
#if HAS_MASS_STORAGE
		parsedFileInfo.lastModifiedTime = MassStorage::GetLastModifiedTime(filePath);
#endif

		parsedFileInfo.isValid = true;

		// If the file is empty or not a G-Code file, we don't need to parse anything
		constexpr const char *_ecv_array GcodeFileExtensions[] = { ".gcode", ".g", ".gco", ".gc", ".nc" };
		bool isGcodeFile = false;
		for (const char *_ecv_array ext : GcodeFileExtensions)
		{
			if (StringEndsWithIgnoreCase(filePath, ext))
			{
				isGcodeFile = true;
				break;
			}
		}

		if (!isGcodeFile || fileBeingParsed->Length() == 0)
		{
			fileBeingParsed->Close();
			parsedFileInfo.incomplete = false;
			info = parsedFileInfo;
			return GCodeResult::ok;
		}

		parseState = FileParseState::parsingHeader;
		scanStartOffset = GCodeOverlapSize;
		numThumbnailsStored = 0;
		parsedFileInfo.numFilaments = 0;
		atLineStart = true;
		foundHeightComment = false;

		trailerBytesProcessed = 0;
		prepTime = millis() - now;
		accumulatedReadTime = accumulatedParseTime = accumulatedSeekTime = 0;
	}

	// Getting file information take a few runs. Speed it up when we are not printing by calling it several times.
	vars = customVariables;
	const uint32_t loopStartTime = millis();
	do
	{
		switch (parseState)
		{
		case FileParseState::parsingHeader:
			{
				bool reachedEnd;
				if (!ReadAndProcessFileChunk(true, reachedEnd))
				{
					reprap.GetPlatform().MessageF(WarningMessage, "Failed to read header of G-Code file \"%s\"\n", filePath);
					parseState = FileParseState::notParsing;
					fileBeingParsed->Close();
					info = parsedFileInfo;
					return GCodeResult::warning;
				}

				if (reachedEnd)
				{
					if (parsedFileInfo.objectHeight == 0.0 && parsedFileInfo.layerHeight != 0.0 && parsedFileInfo.numLayers != 0)
					{
						parsedFileInfo.objectHeight = parsedFileInfo.layerHeight * parsedFileInfo.numLayers;
					}
					else if (parsedFileInfo.objectHeight > 0.0 && parsedFileInfo.layerHeight == 0.0 && parsedFileInfo.numLayers != 0)		// Fusion 3 gives us height and number of layers in the header but not layer height
					{
						parsedFileInfo.layerHeight = parsedFileInfo.objectHeight/parsedFileInfo.numLayers;
					}
					parseState = FileParseState::seeking;
				}
			}

			break;

		case FileParseState::seeking:
			if (FindEndComments())
			{
				parseState = FileParseState::parsingFooter;
			}
			else
			{
				reprap.GetPlatform().MessageF(WarningMessage, "Could not find footer comments in file \"%s\"\n", filePath);
				parseState = FileParseState::notParsing;
				fileBeingParsed->Close();
				info = parsedFileInfo;
				return GCodeResult::warning;
			}
			break;

		case FileParseState::parsingFooter:
			{
				bool reachedEnd;
				if (!ReadAndProcessFileChunk(false, reachedEnd))
				{
					reprap.GetPlatform().MessageF(WarningMessage, "Failed to read footer from G-Code file \"%s\"\n", filePath);
					parseState = FileParseState::notParsing;
					fileBeingParsed->Close();
					info = parsedFileInfo;
					return GCodeResult::warning;
				}

				if (reachedEnd)
				{
					if (reprap.Debug(Module::PrintMonitor))
					{
						reprap.GetPlatform().MessageF(UsbMessage, "Parsing complete, processed %lu header bytes and %lu trailer bytes, prep time %.3fs, read time %.3fs, parse time %.3fs, seek time %.3fs\n",
											parsedFileInfo.headerSize, trailerBytesProcessed,
											(double)((float)prepTime/1000.0), (double)((float)accumulatedReadTime/1000.0), (double)((float)accumulatedParseTime/1000.0), (double)((float)accumulatedSeekTime/1000.0));
					}
					parseState = FileParseState::notParsing;
					fileBeingParsed->Close();
					if (parsedFileInfo.numLayers == 0 && parsedFileInfo.layerHeight > 0.0 && parsedFileInfo.objectHeight > 0.0)
					{
						parsedFileInfo.numLayers = lrintf(parsedFileInfo.objectHeight / parsedFileInfo.layerHeight);
					}
					parsedFileInfo.incomplete = false;
					info = parsedFileInfo;
					return GCodeResult::ok;
				}
			}
			break;

		default:	// should not get here
			parsedFileInfo.incomplete = false;
			fileBeingParsed->Close();
			info = parsedFileInfo;
			parseState = FileParseState::notParsing;
			return GCodeResult::ok;
		}
		lastFileParseTime = millis();
	} while (!reprap.GetPrintMonitor().IsPrinting() && lastFileParseTime - loopStartTime < MaxFileinfoProcessTime);

	if (quitEarly)
	{
		info = parsedFileInfo;				// note that the 'incomplete' flag is still set
		fileBeingParsed->Close();
		parseState = FileParseState::notParsing;
		return GCodeResult::ok;
	}
	return GCodeResult::notFinished;
}

// Read and process a chunk of the file.
// On entry, fileOverlapLength is the offset into the start of the buffer that we should read into, and if it is nonzero then there was an incomplete comment line already at the start of the buffer.
// The file position is correct for reading the next chunk of the file.
// If successful set reachedEnd true if we ran out of data in the file; otherwise set reachedEnd false and leave the buffer, fileOverlaplength and the file seek position ready for the next call to this function.
// Return true if success, false if there was a file read error.
bool FileInfoParser::ReadAndProcessFileChunk(bool isParsingHeader, bool& reachedEnd) noexcept
{
	// Read a chunk of the file into the buffer after the data we have already.
	// For efficiency, read complete 512byte sectors on 512byte sector boundaries, and read them into 32-bit aligned memory - that allows the SD card driver to DMA directly into the buffer.
	bufferStartFilePosition = fileBeingParsed->Position() - GCodeOverlapSize;					// we need to keep this up to date so that we can record the file offsets of thumbnails
	const FilePosition sizeLeft = fileBeingParsed->Length() - fileBeingParsed->Position();
	const size_t sizeToRead = (size_t)min<FilePosition>(sizeLeft, GCodeReadSize);

	const uint32_t now1 = millis();
	const int nbytes = fileBeingParsed->Read(buf + GCodeOverlapSize, sizeToRead);
	accumulatedReadTime += millis() - now1;

	if (nbytes != (int)sizeToRead)
	{
		return false;
	}

	if (!isParsingHeader)
	{
		trailerBytesProcessed += (unsigned int)nbytes;
	}

	const char *_ecv_array bufp = buf + scanStartOffset;
	char *_ecv_array bufLim = buf + GCodeOverlapSize + (unsigned int)nbytes;
	reachedEnd = (sizeLeft <= GCodeReadSize);
	if (reachedEnd)
	{
		// This is the last read of the file, so append a '\n' terminator so that ScanBuffer can process the last line
		*bufLim = '\n';
		++bufLim;
	}

	if (!atLineStart)
	{
		// The last scan encountered a very long line. Skip the rest of it before resuming parsing.
		while (bufp < bufLim)
		{
			const char c = *bufp++;
			if (c == '\n' || c == '\r')
			{
				atLineStart = true;
				break;
			}
		}
	}

	const uint32_t now2 = millis();
	const char *_ecv_array const pEnd = (bufp == bufLim) ? bufp : ScanBuffer(bufp, bufLim, isParsingHeader, reachedEnd);
	accumulatedParseTime += millis() - now2;

	if (!reachedEnd && pEnd < bufLim)
	{
		scanStartOffset = GCodeOverlapSize - (bufLim - pEnd);
		memcpy(buf + scanStartOffset, pEnd, bufLim - pEnd);
	}
	else
	{
		scanStartOffset = GCodeOverlapSize;
	}
	if (reachedEnd && isParsingHeader)
	{
		parsedFileInfo.headerSize = bufferStartFilePosition + (pEnd - buf);
	}

	return true;
}

// Scan the buffer for data we are interested in.
// On entry, pStart is at the start of a line of the file.
// Return a pointer to the incomplete comment line at the end, if there is one, or pEnd if there isn't.
// If stopOnGCode is set then if we reach a line of GCode, set 'stopped'; otherwise leave 'stopped' alone.
const char *_ecv_array FileInfoParser::ScanBuffer(const char *_ecv_array pStart, const char *_ecv_array pEnd, bool isParsingHeader, bool& stopped) noexcept
{
	while (true)
	{
		// Skip any blank lines or additional line end characters
		while (pStart < pEnd && (*pStart == '\r' || *pStart == '\n')) { ++pStart; }

		const char *_ecv_array lineStart = pStart;
		const char *_ecv_array lineEnd = pStart;

		// Find the end of the line
		while (lineEnd < pEnd && *lineEnd != '\r' && *lineEnd != '\n')
		{
			++lineEnd;
		}

		if (lineEnd == pEnd)
		{
			// The line ending is not within the buffer
			if (lineStart >= buf + GCodeReadSize)
			{
				// This line starts within the last GCODE_OVERLAP_SIZE of the buffer, so we can safely leave processing it until the next bufferfull
				return lineStart;
			}

			// This line is longer than GCODE_OVERLAP_SIZE. Ignore it and return, flagging that we are not at a line start.
			atLineStart = false;
			return pEnd;
		}

		char c = *pStart++;
		switch (c)
		{
		case ';':
			// Found a whole-line comment
			{
				while (pStart < pEnd && ((c = *pStart) == ' ' || c == '-'))
				{
					++pStart;										// skip spaces and dashes after the leading semicolon
				}

				if (isAlpha(c))										// all keywords we are interested in start with a letter
				{
					const char *_ecv_array kStart = pStart;			// save keyword start for later

					// If we are not parsing the header and we can see that there is a G- or M-command after this comment, save time by not parsing the comment.
					// This saves time by not processing most comments in the GCode file when we haven't yet reached the footer.
					if (isParsingHeader || pStart == pEnd || (*pEnd != 'G' && *pEnd != 'M'))
					{
						// pStart now points to the line terminator and kStart to the possible start of a key phrase.
						// There is definitely a line terminator, and as line terminators do not occur in key phrases, it is safe to call StringStartsWith
						// Do a binary search of the table on the first character
						size_t low = 0, high = ARRAY_SIZE(parseTable);
						const char c1 = toupper(c);
						do
						{
							size_t mid = (low + high)/2;
							if (c1 < parseTable[mid].key[0])
							{
								high = mid;
							}
							else if (c1 > parseTable[mid].key[0])
							{
								low = mid + 1;
							}
							else
							{
								// Found a key phrase that starts with the same letter. Find the first such phrase.
								while (mid != 0 && parseTable[mid - 1].key[0] == c1)
								{
									--mid;
								}
								do
								{
									const ParseTableEntry& pte = parseTable[mid];
									if (StringStartsWith(kStart + 1, pte.key + 1))
									{
										// Found the key phrase. Check for a separator after it unless the key phrase ends with '#'.
										const char *_ecv_array argStart = kStart + strlen(pte.key);
										if (*(argStart - 1) != '#')
										{
											char c2 = *argStart;
											if (c2 != ' ' && c2 != '\t' && c2 != ':' && c2 != '=' && c2 != ',')
											{
												break;
											}

											// Skip further separators
											do
											{
												++argStart;
											} while ((c2 = *argStart) == ' ' || c2 == '\t' || c2 == ':' || c2 == '=');
										}
										(this->*pte.FileInfoParser::ParseTableEntry::func)(kStart, argStart, lineEnd, pte.param);
										break;
									}
									++mid;
								} while (mid < ARRAY_SIZE(parseTable) && c1 == parseTable[mid].key[0]);
								break;
							}
						} while (low + 1 < high);
						// If we get here then there is no phrase in the table that starts with the first letter of the comment, or we have found one and processed it
					}
				}
			}
			break;

		case 'G':
			if (!isParsingHeader && !foundHeightComment)
			{
				// If it's a G0 or G1 Z command with Z then record the height (note that Cura uses e.g. "G0 X121.297 Y73.506 Z17.7" when changing layers)
				if ((*pStart == '1' || *pStart == '0') && !isDigit(pStart[1]))
				{
					++pStart;
					while (pStart != lineEnd && *pStart != 'Z' && *pStart != ';') { ++pStart; }
					if (*pStart == 'Z')
					{
						const char *_ecv_array q;
						const float height = SafeStrtof(pStart + 1, &q);
						if (!std::isnan(height) && !std::isinf(height) && height > parsedFileInfo.objectHeight)
						{
							// If the Z movement command ends in ";E or "; E" then ignore it
							while (q < lineEnd && *q != ';') { ++q; }
							if (*q != ';' || (q[1] != 'E' && (q[1] != ' ' || q[2] != 'E')))
							{
								parsedFileInfo.objectHeight = height;
							}
						}
					}
				}
				break;
			}
			// no break
		case 'M':
		case 'T':
			if (isParsingHeader)
			{
				stopped = true;
				return lineStart;
			}
			break;

		default:
			break;
		}

		// Skip the line ending
		pStart = lineEnd + 1;
	}
}

// Find the starting position of the file end comments, get the object height, set up the buffer ready to parse them.
// Return true if successful.
bool FileInfoParser::FindEndComments() noexcept
{
	// Temporary code until we find something better
	const FilePosition roundedDownLength = fileBeingParsed->Length() & ~(GCodeReadSize - 1);		// round file length down to a multiple of the read size
	FilePosition pos;
	if (roundedDownLength > parsedFileInfo.headerSize + GCodeFooterSize)
	{
		// Usual case when the file is long
		pos = roundedDownLength - GCodeFooterSize;
		scanStartOffset = GCodeOverlapSize;
	}
	else
	{
		// Start scanning from just after the header
		pos = parsedFileInfo.headerSize & ~(GCodeReadSize - 1);
		scanStartOffset = (parsedFileInfo.headerSize & (GCodeReadSize - 1)) + GCodeOverlapSize;
	}
	atLineStart = false;
	const uint32_t now = millis();
	const bool ret = fileBeingParsed->Seek(pos);
	accumulatedSeekTime += millis() - now;
	return ret;
}

// Parse table entry methods

// Process the slicer name and (if present) version
void FileInfoParser::ProcessGeneratedBy(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept
{
	const char *_ecv_array introString = "";
	switch (param)
	{
	case 1:				// Cura (old)
		introString = "Cura at ";
		break;

	case 2:				// Kisslicer and Fusion 360 - the keyword is the generator name
		p = k;
		break;

	default:
		break;
	}

	parsedFileInfo.generatedBy.copy(introString);
	while (*p >= ' ')
	{
		parsedFileInfo.generatedBy.cat(*p++);
	}
}

// Process the layer height
void FileInfoParser::ProcessLayerHeight(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept
{
	const char *tailPtr;
	const float val = SafeStrtof(p, &tailPtr);
	if (tailPtr != p && !std::isnan(val) && !std::isinf(val))	// if we found and converted a number
	{
		parsedFileInfo.layerHeight = val;
	}
}

void FileInfoParser::ProcessObjectHeight(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept
{
	const char *tailPtr;
	const float val = SafeStrtof(p, &tailPtr);
	if (tailPtr != p && !std::isnan(val) && !std::isinf(val))	// if we found and converted a number
	{
		parsedFileInfo.objectHeight = val;
		foundHeightComment = true;
	}
}

// Process the number of layers
void FileInfoParser::ProcessNumLayers(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept
{
	const unsigned int val = StrToU32(p);
	if (val > 0)
	{
		parsedFileInfo.numLayers = val;
	}
}

// Process the estimated job time
void FileInfoParser::ProcessJobTime(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept
{
	const char *_ecv_array const q = p;
	float days = 0.0, hours = 0.0, minutes = 0.0;
	float secs = SafeStrtof(q, &p);
	if (q != p)
	{
		while (*p == ' ')
		{
			++p;
		}
		if (*p == ':')											// special code for REALvision
		{
			minutes = secs;
			secs = SafeStrtof(p + 1, &p);
			if (*p == ':')
			{
				hours = minutes;
				minutes = secs;
				secs = SafeStrtof(p + 1, &p);
				// I am assuming that it stops at hours
			}
		}
		else
		{
			if (*p == 'd')
			{
				days = secs;
				if (StringStartsWithIgnoreCase(p, "day"))		// not sure if any slicer needs this, but include it j.i.c.
				{
					p += 3;
					if (*p == 's')
					{
						++p;
					}
				}
				else
				{
					++p;
				}
				while (*p == ' ' || *p == ':')					// Fusion 360 gives e.g. ";Print time: 40m:36s"
				{
					++p;
				}
				secs = SafeStrtof(p, &p);
			}
			if (*p == 'h')
			{
				hours = secs;
				if (StringStartsWithIgnoreCase(p, "hour"))		// S3D
				{
					p += 4;
					if (*p == 's')
					{
						++p;
					}
				}
				else
				{
					++p;
				}
				while (*p == ' ' || *p == ':')					// Fusion 360 gives e.g. ";Print time: 40m:36s"
				{
					++p;
				}
				secs = SafeStrtof(p, &p);
			}
			if (*p == 'm')
			{
				minutes = secs;
				if (StringStartsWithIgnoreCase(p, "minute"))
				{
					p += 6;
					if (*p == 's')
					{
						++p;
					}
				}
				else if (StringStartsWithIgnoreCase(p, "min"))	// Fusion 360
				{
					p += 3;
				}
				else
				{
					++p;
				}
				while (*p == ' ' || *p == ':')					// Fusion 360 gives e.g. ";Print time: 40m:36s"
				{
					++p;
				}
				secs = SafeStrtof(p, &p);
			}
		}
	}
	parsedFileInfo.printTime = max<uint32_t>(lrintf(((days * 24.0 + hours) * 60.0 + minutes) * 60.0 + secs), 1);		// if print time is zero, call it 1 second to prevent it being shown as "n/a"
}

// Process the simulated time
void FileInfoParser::ProcessSimulatedTime(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept
{
	const char *_ecv_array const q = p;
	const uint32_t secs = StrToU32(p, &p);
	if (q != p)
	{
		parsedFileInfo.simulatedTime = secs;
	}
}

// Process a thumbnail
// The overlap is small enough that we can discount the possibility of finding more than one thumbnail header in the overlap area.
// Thumbnail data is preceded by comment lines of the following form:
//	; thumbnail_QOI begin 32x32 2140
// or
//	; thumbnail begin 32x32 2140

void FileInfoParser::ProcessThumbnail(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept
{
	if (numThumbnailsStored == MaxThumbnails)
	{
		return;		// no more space to store thumbnail info
	}

	GCodeFileInfo::ThumbnailInfo::ImageFormat fmt = (param == 2) ? GCodeFileInfo::ThumbnailInfo::ImageFormat::jpeg
													: (param == 1) ? GCodeFileInfo::ThumbnailInfo::ImageFormat::qoi
														:GCodeFileInfo::ThumbnailInfo::ImageFormat::png;
	// Store this thumbnail data
	const char *_ecv_array npos;
	const uint32_t w = StrToU32(p, &npos);
	if (w >= 16 && w <= 500 && *npos == 'x')
	{
		p = npos + 1;
		const uint32_t h = StrToU32(p, &npos);
		if (h >= 16 && h <= 500 && *npos == ' ')
		{
			p = npos + 1;
			const uint32_t size = StrToU32(p, &npos);
			if (size >= 10)
			{
				const FilePosition offset = bufferStartFilePosition + (lineEnd + 1 - buf);
				GCodeFileInfo::ThumbnailInfo& th = parsedFileInfo.thumbnails[numThumbnailsStored++];
				th.width = w;
				th.height = h;
				th.size = size;
				th.format = fmt;
				th.offset = offset;
			}
		}
	}
}

// Scan the buffer for a 2-part filament used string. Return the number of filament found.
void FileInfoParser::ProcessFilamentUsedEmbedded(const char *_ecv_array p, const char *_ecv_array s2) noexcept
{
	const char *_ecv_array q;
	const uint32_t num = StrToU32(p, &q);
	if (q != p && num < MaxFilaments && StringStartsWith(q, s2))
	{
		p = q + strlen(s2);
		while (strchr(" :\t", *p) != nullptr)
		{
			++p;	// this allows for " Used: "
		}
		if (isDigit(*p))
		{
			const float filamentLength = SafeStrtof(p, nullptr);
			if (!std::isnan(filamentLength) && !std::isinf(filamentLength))
			{
				parsedFileInfo.filamentNeeded[parsedFileInfo.numFilaments++] = filamentLength;
			}
		}
	}
}

// Process filament usage comment
void FileInfoParser::ProcessFilamentUsed(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept
{
	if (parsedFileInfo.numFilaments < MaxFilaments)
	{
		switch (param)
		{
		case 0:
			while (isDigit(*p) && parsedFileInfo.numFilaments < MaxFilaments)
			{
				const char *_ecv_array q = p;
				const float filamentLength = SafeStrtof(q, &p);
				if (!std::isnan(filamentLength) && !std::isinf(filamentLength))
				{
					parsedFileInfo.filamentNeeded[parsedFileInfo.numFilaments] = filamentLength;
					while (*p == ' ') { ++p; }
					if (*p == 'm')
					{
						++p;
						if (*p == 'm')
						{
							++p;
						}
						else
						{
							parsedFileInfo.filamentNeeded[parsedFileInfo.numFilaments] *= 1000.0;		// Cura outputs filament used in metres not mm
						}
					}
					++parsedFileInfo.numFilaments;
				}
				while (strchr(", \t", *p) != nullptr)
				{
					++p;
				}
			}
			break;

		case 1:			// S3D, units are always mm
			if (isDigit(*p))
			{
				const float filamentLength = SafeStrtof(p, nullptr);
				if (!std::isnan(filamentLength) && !std::isinf(filamentLength))
				{
					parsedFileInfo.filamentNeeded[parsedFileInfo.numFilaments] = filamentLength;
					++parsedFileInfo.numFilaments;
				}
			}
			break;

		case 2:			// Kisslicer newer versions
			if (*p == '#')
			{
				++p;				// later KISSlicer versions add a # here
			}
			while (isDigit(*p))
			{
				++p;
			}
			while (strchr(" :=\t", *p) != nullptr)
			{
				++p;
			}

			if (isDigit(*p))
			{
				const float filamentLength = SafeStrtof(p, nullptr);
				if (!std::isnan(filamentLength) && !std::isinf(filamentLength))
				{
					parsedFileInfo.filamentNeeded[parsedFileInfo.numFilaments] = filamentLength;
					++parsedFileInfo.numFilaments;
				}
			}
			break;

		case 3:			// Kisslicer older versions
			// Special case: Old KISSlicer and Pathio only generate the filament volume, so we need to calculate the length from it
			if (reprap.GetPlatform().GetFilamentWidth() > 0.0)
			{
				const float filamentCMM = SafeStrtof(p, nullptr) * 1000.0;
				if (!std::isnan(filamentCMM) && !std::isinf(filamentCMM))
				{
					parsedFileInfo.filamentNeeded[parsedFileInfo.numFilaments++] = filamentCMM / (Pi * fsquare(reprap.GetPlatform().GetFilamentWidth() * 0.5));
				}
			}
			break;

		case 4:			// Ideamaker e.g. ";Material#1 Used: 868.0"
			ProcessFilamentUsedEmbedded(p, " Used");
			break;

		case 5:			// Fusion 360 e.g. ";Extruder 1 material used: 1811mm"
			ProcessFilamentUsedEmbedded(p, " material used");
			break;
		}
	}
}

void FileInfoParser::ProcessCustomInfo(const char *_ecv_array k, const char *_ecv_array p, const char *_ecv_array lineEnd, int param) noexcept
{
	if (vars != nullptr && isAlpha(*p))
	{
		const char *_ecv_array kStart = p;
		do
		{
			++p;
		} while (isAlnum(*p) || *p == '_');
		const size_t nameLength = p - kStart;

		while (*p == ' ' || *p == '\t') { ++p; }
		if (*p == '=')
		{
			++p;
			ExpressionParser parser(nullptr, p, lineEnd);
			ExpressionValue ev;
			try
			{
				ev = parser.Parse(true);
			}
			catch (GCodeException& exc)
			{
				ev.SetNull(nullptr);
				if (reprap.Debug(Module::PrintMonitor))
				{
					exc.DebugPrint();
				}
			}

			auto vset = vars->GetForWriting();
			if (vset->Lookup(kStart, nameLength, false) == nullptr)
			{
				vset->InsertNew(kStart, nameLength, ev, 0);
			}
		}
	}
}

#endif

// End
