# ariac_traj_sender

Start of node to convert vacuum-gripper poses into corresponding joint-space solutions
and send these off as part of trajectory messages.

At present, uses input joint values to compute a reachable gripper pose.
Then invokes IK to find closest joint-space pose (recovering original jspace pose).

Stuffs jspace poses into a trajectory message and publishes this message to move the arm.

Will want to hard-code some key poses.

## Example usage
rosrun ariac_traj_sender traj_sender 

## Running tests/demos
    
