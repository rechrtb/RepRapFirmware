/*
 * StallDetectionEndstop.cpp
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#include "StallDetectionEndstop.h"

#if HAS_STALL_DETECT || SUPPORT_CAN_EXPANSION

#include <Platform/RepRap.h>
#include <Movement/Move.h>
#include <Movement/Kinematics/Kinematics.h>

// Stall detection endstop constructor, used for axis endstops
StallDetectionEndstop::StallDetectionEndstop(uint8_t p_axis, EndStopPosition pos, bool p_individualMotors) noexcept
	: Endstop(p_axis, pos), individualMotors(p_individualMotors)
{
}

// Constructor used for extruder endstops
StallDetectionEndstop::StallDetectionEndstop() noexcept
	: Endstop(NO_AXIS, EndStopPosition::noEndStop), individualMotors(false), stopAll(true)
{
}

// Test whether we are at or near the stop
bool StallDetectionEndstop::Stopped() const noexcept
{
	//TODO account for CAN-connected stalled drivers
#if HAS_STALL_DETECT
	return GetStalledDrivers(localDriversMonitored).IsNonEmpty();
#else
	return false;
#endif
}

// This is called to prime axis endstops
bool StallDetectionEndstop::Prime(const Kinematics &_ecv_from kin, const AxisDriversConfig& axisDrivers) noexcept
{
	// Find which drivers are relevant, and decide whether we stop just the driver, just the axis, or everything
	const LogicalDrivesBitmap logicalDrivesToMonitor = kin.GetControllingDrives(GetAxis(), true);
	stopAll = logicalDrivesToMonitor.Intersects(~LogicalDrivesBitmap::MakeFromBits(GetAxis()));

	// Build the lists of local and remote drivers to monitor
	localDriversMonitored.Clear();
#if SUPPORT_CAN_EXPANSION
	remoteDriversMonitored.Clear();
#endif
	numDriversLeft = 0;
	logicalDrivesToMonitor.Iterate([this](unsigned int drive, unsigned int count)
									{
										const AxisDriversConfig& config = reprap.GetMove().GetAxisDriversConfig(drive);
										for (size_t i = 0; i < config.numDrivers; ++i)
										{
											const DriverId did = config.driverNumbers[i];
#if SUPPORT_CAN_EXPANSION
											if (did.IsRemote())
											{
												AddRemoteDriverToMonitoredList(did);
											}
											else
#endif
											{
												localDriversMonitored.SetBit(did.localDriver);
											}
											++numDriversLeft;
										}
									}
								  );

	//TODO if there any remote stall endstops, check they are set up to report

	return true;
}

#if SUPPORT_CAN_EXPANSION

// Add a remote driver to the list of remote drivers monitored. We maintain a bitmap of local drivers monitored on each relevant CAN-connected board.
void StallDetectionEndstop::AddRemoteDriverToMonitoredList(DriverId did) noexcept
{
	for (size_t i = 0; i < remoteDriversMonitored.Size(); ++i)
	{
		if (remoteDriversMonitored[i].boardId == did.boardAddress)
		{
			remoteDriversMonitored[i].driversMonitored.SetBit(did.localDriver);
			return;
		}
	}

	(void)remoteDriversMonitored.Add(RemoteDriversMonitored(did.boardAddress, Bitmap<uint8_t>::MakeFromBits(did.localDriver)));		// we don't expect the vector to overflow
}

#endif

// Check whether the endstop is triggered and return the action that should be performed. Called from the step ISR.
// Note, the result will not necessarily be acted on because there may be a higher priority endstop!
EndstopHitDetails StallDetectionEndstop::CheckTriggered() noexcept
{
	EndstopHitDetails rslt;				// initialised by default constructor
#if HAS_STALL_DETECT
	const LocalDriversBitmap relevantStalledDrivers = GetStalledDrivers(localDriversMonitored);
	if (relevantStalledDrivers.IsNonEmpty())
	{
		rslt.axis = GetAxis();
		if (rslt.axis == NO_AXIS)
		{
			rslt.SetAction(EndstopHitAction::stopAll);
		}
		else if (stopAll)
		{
			rslt.SetAction(EndstopHitAction::stopAll);
			if (GetAtHighEnd())
			{
				rslt.setAxisHigh = true;
			}
			else
			{
				rslt.setAxisLow = true;
			}
		}
		else if (individualMotors && numDriversLeft > 1)
		{
			rslt.SetAction(EndstopHitAction::stopDriver);
# if SUPPORT_CAN_EXPANSION
			rslt.driver.boardAddress = 0;
# endif
			rslt.driver.localDriver = relevantStalledDrivers.LowestSetBit();
		}
		else
		{
			rslt.SetAction(EndstopHitAction::stopAxis);
			if (GetAtHighEnd())
			{
				rslt.setAxisHigh = true;
			}
			else
			{
				rslt.setAxisLow = true;
			}
		}
		return rslt;
	}
#endif

#if SUPPORT_CAN_EXPANSION
	//TODO account for CAN-connected drivers
#endif
	return rslt;
}

// This is called by the ISR to acknowledge that it is acting on the return from calling CheckTriggered. Called from the step ISR.
// Return true if we have finished with this endstop or probe in this move.
bool StallDetectionEndstop::Acknowledge(EndstopHitDetails what) noexcept
{
	switch (what.GetAction())
	{
	case EndstopHitAction::stopAll:
	case EndstopHitAction::stopAxis:
		return true;

	case EndstopHitAction::stopDriver:
		localDriversMonitored.ClearBit(what.driver.localDriver);
		--numDriversLeft;
		return false;

	default:
		return false;
	}
}

void StallDetectionEndstop::AppendDetails(const StringRef& str) noexcept
{
	str.cat((individualMotors) ? "motor stall (individual motors)" : "motor stall (any motor)");
}

void StallDetectionEndstop::SetDrivers(LocalDriversBitmap extruderDrivers) noexcept
{
	localDriversMonitored = extruderDrivers;
	stopAll = true;
}

EndstopValidationResult StallDetectionEndstop::Validate(const DDA& dda, uint8_t& failingDriver) const noexcept
{
#if HAS_STALL_DETECT
	const float speed = fabsf(dda.GetMotorTopSpeed(GetAxis()));
	return reprap.GetMove().CheckStallDetectionEnabled(GetAxis(), speed, failingDriver);
#else
	return EndstopValidationResult::ok;
#endif
}

#endif

// End
