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

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanInterface.h>
#endif

// Stall detection endstop constructor, used for axis endstops
StallDetectionEndstop::StallDetectionEndstop(uint8_t p_axis, EndStopPosition pos, bool p_individualMotors) noexcept
	: Endstop(p_axis, pos),
#if SUPPORT_CAN_EXPANSION
	  newStallReported(false),
#endif
	  individualMotors(p_individualMotors)
{
}

// Constructor used for extruder endstops
StallDetectionEndstop::StallDetectionEndstop() noexcept
	: Endstop(NO_AXIS, EndStopPosition::noEndStop),
#if SUPPORT_CAN_EXPANSION
	  newStallReported(false),
#endif
	  individualMotors(false), stopAll(true)
{
}

// Test whether we are at or near the stop
bool StallDetectionEndstop::Stopped() const noexcept
{
	//TODO account for CAN-connected stalled drivers
#if SUPPORT_CAN_EXPANSION
	return
# if HAS_STALL_DETECT
		GetStalledDrivers(localDriversMonitored).IsNonEmpty() ||
# endif
		newStallReported;
#else		// must have HAS_STALL_DETECT
	return GetStalledDrivers(localDriversMonitored).IsNonEmpty();
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
	logicalDrivesToMonitor.Iterate([this](unsigned int drive, unsigned int count) noexcept
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

#if !HAS_STALL_DETECT
	if (localDriversMonitored.IsNonEmpty())
	{
		return false;					// a local driver is involved but it doesn't support stall detection
	}
#endif

#if SUPPORT_CAN_EXPANSION
	newStallReported = false;

	// If there any remote stall endstops, set them up to report
	size_t failedIndex;
	const bool ok = remoteDriversMonitored.IterateWhile([&failedIndex](const RemoteDriversMonitored& entry, size_t index) noexcept -> bool
														{
															//TODO set up remote handle
															RemoteInputHandle h;
															h.Set(RemoteInputHandle::typeStallEndstop, 0, 0);
															String<1> dummy;
															bool dummyB;
															const bool ok = CanInterface::CreateHandle(entry.boardId, h, "", entry.driversMonitored.GetRaw(), 0, dummyB, dummy.GetRef()) == GCodeResult::ok;
															if (!ok)
															{
																failedIndex = index;
															}
															return ok;
														}
													   );
	if (!ok)
	{
		// Delete any remote handles that we set up
		RemoteInputHandle h;
		h.Set(RemoteInputHandle::typeStallEndstop, 0, 0);
		String<1> dummy;
		for (size_t i = 0; i < failedIndex; ++i)
		{
			(void)CanInterface::DeleteHandle(remoteDriversMonitored[i].boardId, h, dummy.GetRef());
		}
	}
	return ok;
#else
	return true;
#endif
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

	(void)remoteDriversMonitored.Add(RemoteDriversMonitored(did.boardAddress, RemoteDriversBitmap::MakeFromBits(did.localDriver)));		// we don't expect the vector to overflow
}

#endif

// Construct and return a result object
EndstopHitDetails StallDetectionEndstop::GetResult(
#if SUPPORT_CAN_EXPANSION
													CanAddress boardAddress,
#endif
													unsigned int driverWithinBoard) noexcept
{
	EndstopHitDetails rslt;
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
#if SUPPORT_CAN_EXPANSION
		rslt.driver.boardAddress = boardAddress;
#endif
		rslt.driver.localDriver = driverWithinBoard;
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

// Check whether the endstop is triggered and return the action that should be performed. Called from the step ISR.
// Note, the result will not necessarily be acted on because there may be a higher priority endstop!
EndstopHitDetails StallDetectionEndstop::CheckTriggered() noexcept
{
#if HAS_STALL_DETECT
	// Check for local stalled drivers first
	const LocalDriversBitmap relevantStalledDrivers = GetStalledDrivers(localDriversMonitored);
	if (relevantStalledDrivers.IsNonEmpty())
	{
		return GetResult(
#if SUPPORT_CAN_EXPANSION
							CanInterface::GetCanAddress(),
#endif
							relevantStalledDrivers.LowestSetBit());
	}
#endif

#if SUPPORT_CAN_EXPANSION
	// Account for CAN-connected drivers
	if (newStallReported.exchange(false))
	{
		// Find the board/driver that has stalled
		for (size_t i = 0; i < remoteDriversMonitored.Size(); ++i)
		{
			const RemoteDriversMonitored& elem = remoteDriversMonitored[i];
			const RemoteDriversBitmap stalledRemoteDrives = elem.driversMonitored & elem.driversStalled;
			if (stalledRemoteDrives.IsNonEmpty())
			{
				newStallReported = true;					// there may be more than one stalled drive reported, so make sure we check again
				return GetResult(elem.boardId, stalledRemoteDrives.LowestSetBit());
			}
		}
	}
#endif
	return EndstopHitDetails();
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
#if SUPPORT_CAN_EXPANSION
		if (what.driver.boardAddress != CanInterface::GetCanAddress())
		{
			for (size_t i = 0; i < remoteDriversMonitored.Size(); ++i)
			{
				RemoteDriversMonitored& elem = remoteDriversMonitored[i];
				if (elem.boardId == what.driver.boardAddress)
				{
					elem.driversMonitored.ClearBit(what.driver.localDriver);
					break;
				}
			}
		}
		else
#endif
		{
			localDriversMonitored.ClearBit(what.driver.localDriver);
		}
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

#if SUPPORT_CAN_EXPANSION

// Record any stalled remote drivers that are meant for us
void StallDetectionEndstop::HandleStalledRemoteDrivers(CanAddress boardAddress, RemoteDriversBitmap driversReportedStalled) noexcept
{
	for (size_t i = 0; i < remoteDriversMonitored.Size(); ++i)
	{
		RemoteDriversMonitored& entry = remoteDriversMonitored[i];
		if (boardAddress == entry.boardId)
		{
			const RemoteDriversBitmap pending = entry.driversMonitored & ~entry.driversStalled;
			const RemoteDriversBitmap newStalls = pending & driversReportedStalled;
			if (newStalls.IsNonEmpty())
			{
				entry.driversStalled |= newStalls;
				newStallReported = true;
			}
			return;
		}
	}
}

#endif

#endif

// End
