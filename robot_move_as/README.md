# robot_move_as

This is an action server to manipulate parts with the UR10.  Send goals consisting of source part and target part, where
"part" includes specification of the part name, its location (e.g. BIN number) and its pose (w/rt world coords).

The MOVE case is the primary case, which handles moving specified parts from source to target.

Special case exists if the location is CONVEYOR, as this requires conveyor tracking.

Another special case is if a part needs to be flipped (inverted). This should only apply to pulley parts.  If the
source and target poses are inverted orientations, then a "flip" operation will be performed, grasping the specified pulley
and placing it approximately at its original coordinates, but flipped.  This operation returns an error code that the
part has been dropped, offering the system the option to resense it with a camera and regrasp it (this time not requiring
a flip operation).

## Example usage
With the simulator running:
`rosrun  robot_move_as robot_move_as`
starts this server.  

## Running tests/demos
A test program to flip pulley parts can be run with:
`rosrun cwru_ariac robot_move_as_part_flip_tester`

This assumes the setup of qualifier 3b, which has 4 pulley parts on BIN8.  The tester will prompt for a pulley code, 0-3, and
it will invoke flipping the indicated pulley.    
