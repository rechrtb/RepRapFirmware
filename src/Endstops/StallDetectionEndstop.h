/*
 * StallDetectionEndstop.h
 *
 *  Created on: 15 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_
#define SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_

#include "Endstop.h"

#if SUPPORT_CAN_EXPANSION
# include <General/Vector.h>
#endif

#if HAS_STALL_DETECT || SUPPORT_CAN_EXPANSION

// Motor stall detection endstop
class StallDetectionEndstop final : public Endstop
{
public:
	DECLARE_FREELIST_NEW_DELETE(StallDetectionEndstop)

	StallDetectionEndstop(uint8_t p_axis, EndStopPosition pos, bool p_individualMotors) noexcept;		// for creating axis endstops
	StallDetectionEndstop() noexcept;																	// for creating the single extruders endstop

	EndStopType GetEndstopType() const noexcept override { return (individualMotors) ? EndStopType::motorStallIndividual : EndStopType::motorStallAny; }
	bool Stopped() const noexcept override;
	bool Prime(const Kinematics &_ecv_from kin, const AxisDriversConfig& axisDrivers) noexcept override;
	EndstopHitDetails CheckTriggered() noexcept override;
	bool Acknowledge(EndstopHitDetails what) noexcept override;
	void AppendDetails(const StringRef& str) noexcept override;
	bool ShouldReduceAcceleration() const noexcept override { return true; }
	EndstopValidationResult Validate(const DDA& dda, uint8_t& failingDriver) const noexcept override;

	void SetDrivers(LocalDriversBitmap extruderDrivers) noexcept;										// for setting which local extruder drives are active extruder endstops

private:
	LocalDriversBitmap localDriversMonitored;
#if SUPPORT_CAN_EXPANSION
	static constexpr size_t MaxRemoteDrivers = max<size_t>(MaxDriversPerAxis, MaxExtrudersPerTool);		// the maximum number of drivers that we may have to monitor
	struct RemoteDriversMonitored																		// struct to represent a remote board and the drivers on it that we are interested in
	{
		CanAddress boardId;
		Bitmap<uint8_t> driversMonitored;

		RemoteDriversMonitored(CanAddress p_boardId, Bitmap<uint8_t> p_driversMonitored)
			: boardId(p_boardId), driversMonitored(p_driversMonitored) { }

		RemoteDriversMonitored() { }
	};
	Vector<RemoteDriversMonitored, MaxRemoteDrivers> remoteDriversMonitored;							// list of relevant remote boards and the drivers we monitor on them

	void AddRemoteDriverToMonitoredList(DriverId did) noexcept;
#endif
	unsigned int numDriversLeft;
	bool individualMotors;
	bool stopAll;
};

#endif

#endif /* SRC_ENDSTOPS_STALLDETECTIONENDSTOP_H_ */
