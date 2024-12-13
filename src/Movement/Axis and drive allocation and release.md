Multiple motion systems in RRF 3.6
==================================

Multiple motion systems present challenges in RRF 3.6 because the final movement is generated using a single set of DriveMovement objects;
whereas in 3.5 the DMs were attached to the DDAs, which were in separate queues for each MS. This is how we handle these channelges in 3.6.

The overall scheme is:
- There is one DDARing for each MS
- Each move generated in a particular MS is put into a DDA in the corresponding DDA ring
- When a move is committed, MoveSegments for the logical drives concerned are generated from the DDA. These segments are attached to the DMs in the common set
- After a DDA is committed, the DDA is used only for tracking when the move should complete, reporting on its parameters (acceleration, top speed etc.), and homing operations
- Each DDARing maintains a copy of endpoints of the last move that was added to it. Only the endpoints of drives that the corresponding MS owns should be considered valid.
- We also maintain a copy of the endpoints of drives that are not owned (lastKnownEndpoints).

To handle allocation of axes and the associated drives:
- Before a MS can do a normal move, the axes mentioned in the corresponding command (or implied in the case of a probing move) must be allocated to that MS.
- Before a MS can do a raw motor move, the drives mentioned in the corresponding command must be allocated to that MS.
- To allocate an axis, we allocate drives that control that axis. We get that set of drives from the kinematics.
- To allocate a set of drives we first check that none of them is already owned by another MS. Then we allocate them to this MS; then copy the lastKnownEndpoints for those drives to the corresponding DDARing;
   then transform the whole set of DDA endpoints to the new machine coordinates of that MS; then transform them to the new user coordinates of that MS.
- Because allocating an axis can change the user position recorded in the MS, we must do the allocation right at the start of processing G0/1/2/3 commands, before we make any use of the initial position.
- To release a set of drives we first copy their last endpoints from the DDARing of the owning MS to lastKnownEndpoints; then we flag those drives as unowned.

Special situations:
- At initialisation time we take the assumed initial machine position for that kinematics, store it in all MSs, transform it to endpoints and store them in lastKnownEndpoints
- When auto-calibrating a delta printer we handle endstop offsets by adjusting the recorded endpoints of the motors. We do this in the DDARing for the MS that did the calibration and also in lastKnownEndpoints.
- When we complete a simulation and restore the user and machine positions, we also need to upate lastKnownEndpoints from the restored position and copy them into the DDARings
- When we home an axis on a Core machine (so homing moves are not raw motor moves), ???
- When we do a probing move, ???
- When we home a drive on a machine that homes using raw motor moves, ???
- When we apply babystepping, ???
- When we pause a print and some moves in the queue are thrown away, ???
- When we do an emergency pause and throw away all moves in the queue, ???
