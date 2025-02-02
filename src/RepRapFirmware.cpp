
/****************************************************************************************************

RepRapFirmware - Main Program

This firmware is intended to be a fully object-oriented highly modular control program for
RepRap self-replicating 3D printers.

It owes a lot to Marlin and to the original RepRap FiveD_GCode.


General design principles:

  * Control by RepRap G Codes.  These are taken to be machine independent, though some may be unsupported.
  * Full use of C++ OO techniques,
  * Make classes hide their data,
  * Make everything except the Platform class (see below) as stateless as possible,
  * No use of conditional compilation except for #include guards - if you need that, you should be
       forking the repository to make a new branch - let the repository take the strain,
  * Concentration of all machine-dependent definitions and code in Platform.h and Platform.cpp,
  * No specials for (X,Y) or (Z) - all movement is 3-dimensional,
  * Except in Platform.h, use real units (mm, seconds etc) throughout the rest of the code wherever possible,
  * Try to be efficient in memory use, but this is not critical,
  * Labour hard to be efficient in time use, and this is critical,
  * Don't abhor floats - they work fast enough if you're clever,
  * Don't avoid arrays and structs/classes,
  * Don't avoid pointers,
  * Use operator and function overloading where appropriate.


Naming conventions:

  * #defines are all CAPITALS_WITH_OPTIONAL_UNDERSCORES_BETWEEN_WORDS
  * No underscores in other names - MakeReadableWithCapitalisation
  * Class names and functions start with a CapitalLetter
  * Variables start with a lowerCaseLetter
  * Use veryLongDescriptiveNames


Structure:

Amongst the main classes are:

  * RepRap
  * GCodes
  * Heat
  * Move
  * Platform
  * Network
  * Webserver
  * PrintMonitor

RepRap:

This is just a container class for the single instances of all the others, and otherwise does very little.

GCodes:

This class is fed GCodes, either from the web interface, or from GCode files, or from a serial interface,
Interprets them, and requests actions from the RepRap machine via the other classes.

Heat:

This class implements all heating and temperature control in the RepRap machine.

Move:

This class controls all movement of the RepRap machine, both along its axes, and in its extruder drives.

Platform:

This is the only class that knows anything about the physical setup of the RepRap machine and its
controlling electronics.  It implements the interface between all the other classes and the RepRap machine.
All the other classes are completely machine-independent (though they may declare arrays dimensioned
to values #defined in Platform.h).

Network:

This class implements a basic TCP interface for the Webserver classes using lwip.

Webserver:

This class talks to the network (via Platform) and implements a simple webserver to give an interactive
interface to the RepRap machine.  It uses the Knockout and Jquery Javascript libraries to achieve this.
In addition, FTP and Telnet servers are provided for easier SD card file management and G-Code handling.

Scanner:
This is an extension meant for 3D scanner boards. Refer to M750 ff. for the exact usage of this module.

PrintMonitor:

This class provides methods to obtain statistics (height, filament usage etc.) from generated G-Code
files and to calculate estimated print end-times for a live print.


When the software is running there is one single instance of each main class, and all the memory allocation is
done on initialization.  new/malloc should not be used in the general running code, and delete is never
used.  Each class has an Init() function that resets it to its boot-up state; the constructors merely handle
that memory allocation on startup.  Calling RepRap.Init() calls all the other Init()s in the right sequence.

There are other ancillary classes that are declared in the .h files for the master classes that use them.  For
example, Move has a DDA class that implements a Bresenham/digital differential analyser.


Timing:

There is a single interrupt chain entered via Platform.Interrupt().  This controls movement step timing, and
this chain of code should be the only place that volatile declarations and structure/variable-locking are
required.  All the rest of the code is called sequentially and repeatedly as follows:

All the main classes have a Spin() function.  These are called in a loop by the RepRap.Spin() function and implement
simple timesharing.  No class does, or ever should, wait inside one of its functions for anything to happen or call
any sort of delay() function.  The general rule is:

  Can I do a thing?
    Yes - do it
    No - set a flag/timer to remind me to do it next-time-I'm-called/at-a-future-time and return.

The restriction this strategy places on almost all the code in the firmware (that it must execute quickly and
never cause waits or delays) is balanced by the fact that none of that code needs to worry about synchronization,
locking, or other areas of code accessing items upon which it is working.  As mentioned, only the interrupt
chain needs to concern itself with such problems.  Unlike movement, heating (including PID controllers) does
not need the fast precision of timing that interrupts alone can offer.  Indeed, most heating code only needs
to execute a couple of times a second.

Most data is transferred bytewise, with classes' Spin() functions typically containing code like this:

  Is a byte available for me?
    Yes
      read it and add it to my buffer
      Is my buffer complete?
         Yes
           Act on the contents of my buffer
         No
           Return
  No
    Return

Note that it is simple to raise the "priority" of any class's activities relative to the others by calling its
Spin() function more than once from RepRap.Spin().

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#include "RepRapFirmware.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>

#include <FreeRTOS.h>
#include <task.h>

// We just need one instance of RepRap; everything else is contained within it and hidden

RepRap reprap;

// Get the format string to use for printing a floating point number to the specified number of significant digits. Zero means the maximum sensible number.
const char *_ecv_array GetFloatFormatString(float val, unsigned int numDigitsAfterPoint) noexcept
{
	// If the value is below 0.1 then use 'g' format and treat the requested number of decimal digits as the number of significant digits needed
	// f the value is very large then use 'g' format and treat the requested number of decimal digits as the number of significant digits needed
	// Else use 'f' format.
	static constexpr const char *_ecv_array FormatStringsF[] = { "%.1f", "%.2f", "%.3f", "%.4f", "%.5f", "%.6f", "%.7f" };
	static constexpr const char *_ecv_array FormatStringsG[] = { "%.1g", "%.2g", "%.3g", "%.4g", "%.5g", "%.6g", "%.7g" };
	static constexpr float MaxValueToDisplayWithAllDecimals[] = { 999999.9, 99999.99, 9999.99, 999, 99.99, 9.99, 0.99 };

	static_assert(ARRAY_SIZE(FormatStringsF) == MaxFloatDigitsDisplayedAfterPoint);
	static_assert(ARRAY_SIZE(FormatStringsG) == MaxFloatDigitsDisplayedAfterPoint);

	constexpr float MinValueToDisplayInFFormat = 0.1;

	if (numDigitsAfterPoint == 0)
	{
		numDigitsAfterPoint = MaxFloatDigitsDisplayedAfterPoint;
	}
	else if (numDigitsAfterPoint > MaxFloatDigitsDisplayedAfterPoint)
	{
		numDigitsAfterPoint = MaxFloatDigitsDisplayedAfterPoint;
	}

	// If the value is small or very large, use 'g' format
	if (fabsf(val) < MinValueToDisplayInFFormat || fabsf(val) > MaxValueToDisplayWithAllDecimals[0])
	{
		return FormatStringsG[numDigitsAfterPoint - 1];
	}

	// Use 'f' format, but don't print more decimal digits than may conceivably be valid
	while (fabsf(val) > MaxValueToDisplayWithAllDecimals[numDigitsAfterPoint - 1])
	{
		--numDigitsAfterPoint;
	}
	// Use 'f' format, but don't print more decimal digits than may conceivably be valid
	return FormatStringsF[numDigitsAfterPoint - 1];
}

//*************************************************************************************************

// Utilities and storage not part of any class

// For debug use
void debugPrintf(const char *_ecv_array fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	if (Platform::HasDebugBuffer())
	{
		vuprintf(Platform::IsrDebugPutc, fmt, vargs);
	}
	else if (__get_BASEPRI() == 0 && !inInterrupt())		// we need both tests to make sure it is safe to write to USB
	{
		reprap.GetPlatform().DebugMessage(fmt, vargs);
	}
	va_end(vargs);
}

// Convert a float to double for passing to printf etc. If it is a NaN or infinity, convert it to 9999.9 to avoid getting JSON parse errors.
float HideNan(float val) noexcept
{
	return (std::isnan(val) || std::isinf(val)) ? 9999.9 : val;
}

// Append a list of driver numbers to a string, with a space before each one
void ListDrivers(const StringRef& str, LocalDriversBitmap drivers) noexcept
{
	drivers.Iterate([&str](unsigned int d, unsigned int) noexcept -> void { str.catf(" %u", d); });
}

// End
