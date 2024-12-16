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
- We also maintain a global copy of the endpoints of drives that are not owned (lastKnownEndpoints). These are set during initialisation and updated whenever a MS releases a drive.

So we have the following sets of endoints:
- The endpoints stored in the DDARings. These are valid only for the drives thsat are owned by the corresponding MS.
- lastKnownEndpoints
- currentMotorPosition maintained by the DM
- 'endpoint' stored in each DDA. These are the planned endpoints for the move that the DDA represents. Homing and probing moves will generally complete before some of those endpoints are reached.

System invariants:
- When no moves are queued and any corrections for homing or calibration have been applied:
-- For drives that are owned, the values for those drives in the DDARing of the owning MS match the motor positions stored in the DMs
-- For drives that are not owned, the values for those drives in lastKnownEndpoints match the motor positions stored in the DMs

To handle allocation of axes and the associated drives:
- Before a MS can do a normal move, the axes mentioned in the corresponding command (or implied in the case of a probing move) must be allocated to that MS.
- Before a MS can do a raw motor move, the drives mentioned in the corresponding command must be allocated to that MS.
- To allocate an axis, we allocate drives that control that axis. We get that set of drives from the kinematics.
- To allocate a set of drives we first check that none of them is already owned by another MS. Then we allocate them to this MS; then copy the lastKnownEndpoints for those drives to the corresponding DDARing;
   then transform the whole set of DDA endpoints to the new machine coordinates of that MS; then transform them to the new user coordinates of that MS.
- Because allocating an axis can change the user position recorded in the MS, we must do the allocation right at the start of processing G0/1/2/3 commands, before we make any use of the initial position.
- To release a set of drives we first copy their last endpoints from the DDARing of the owning MS to lastKnownEndpoints; then we flag those drives as unowned.

Special situations:
- At initialisation time: 
   We take the assumed initial machine position for that kinematics and store it in all MSs
   We transform that machine position to endpoints
   We store those endpoints in lastKnownEndpoints and in the motor positions in the DMs.
- When we execute G92: we allocate the axes (and extruders) that the G92 command refers to, and hence the drives affected.
   We transform the G92 coordinates to machine coordinates and store them in the current MS
   We transform the machine coordinates into drive endpoints
   We store those endpoints in the DDARing for the MS concerned and in the motor positions in the DMs.
- When auto-calibrating a delta printer anbd adjusting positions to take account of endstop offsets:
   We fetch the endpoints from the DDARing for the MS that did the calibration
   We adjust those endpoints as instructed by the calbration
   We also store the new endpoints in lastKnownEndpoints and in the motor positions in the DMs.
- When we complete a simulation and restore the user and machine positions:
   We transform the restored position to machnie coordinates and to endpoints
   We store those endpoints in lastKnownEndpoints from the restored position and copy them into the DDARings
   We also store them in the motor positions in the DMs in case the transformation left us with slightly different endpoints from previously
- When we do an endstop-sensitive move on a Cartesian or Core machine and that axis is controlled by drives that don't affect other axes:
   We only need to stop the axis whose endstop triggered, if other axes are being homes simultaneously
   We retrieve the new endpoint for those drives (usually only one) from the DM motor position, copy them into the DDARing of the owning MS, and also copy them into lastKnownEndpoints
   If it was a homing move then the system will change the machine coordinate of that axis, transform it to endpoints, and update endpoints (similar to executnig a G92 command for that axis)
- When we do an endstop-sensitive move on an axis on a Core machine and that axis is controlled by drives that affect other axes:
   We stop all drives
   We retrieve the new endpoints for all drives owned by the MS that initiated the homing move from the DM motor position, copy them into the DDARing of the owning MS, and also copy them into lastKnownEndpoints
 When we do an endstop-sensitive move on a machine that homes using raw motor moves:
   We stop the drive whose endstop triggered 
   We copy its endpoint from the DM motor position into the DDARing of the MS that initiated the move and into lastKnownEndpoints
- When we do a probing move and the probe triggers:
   We stop all drives
   We retrieve the new endpoints for all drives owned by the MS that initiated the homing move from the DM motor position, copy them into the DDARing of the owning MS, and also copy them into lastKnownEndpoints
 When we apply babystepping:
   Currently babystepping assumes that Z motion and not other motion is controlled only by the Z drive. It adjusts the Z position and the endpoint of the Z driver in existing moves in the queue.
   We must also update the Z drive endpont in the DDARing by the amount of Z babystepping pushed throuh the queue.
- When we pause a print and some moves in the queue are thrown away:
   We must set the endpoint in the DDA ring in which moves have been thrown away to the endpont of the last completed move
- When we do an emergency pause and throw away all moves in the queue:
   If we are definitely going to stop, we needn't update endpoints. Otherwise, we must set the endpoint in the DDA ring in which moves have been thrown away to the endpont of the last completed move.

  Common operations:
  - Changing the deemed positions of some axes/drives, requiring endpoints to be adjusted on ownding DDARings, DMs and lastKnownEndpoints.
     Called during initialisation (but no drives are owned)
     Called by G92 (at least the affected drives are owned)
     Called when homing moves change the assumed machine coordinates
     Called when Z probing moves change the assumed machine coordinates
  - Retrieving from the DMs endpoints of some drives that were stopped by an endstop or probe, updating the owning DDARing and lastKnownEndpoints, and updating axis corodinates
     Called after a endstop-sensitive or probing move that stops one or more drives, before any adjustment to the assumed machine coordinates
  