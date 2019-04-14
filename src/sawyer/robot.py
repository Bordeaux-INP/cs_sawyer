from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

class Robot(object):
    def __init__(self, joint_names):
        self._motion  = MotionTrajectory()
        self.joint_names = joint_names
        
    def execute(self, trajectory):
        self._motion.clear_waypoints()
        self._motion.set_joint_names(self.joint_names)

        for point in trajectory:
            waypoint = MotionWaypoint()
            waypoint.set_joint_angles(point)
            self._motion.append_waypoint(waypoint)
        print(self._motion.to_msg())
        return self._motion.send_trajectory()

    def move_to_joint_positions(self, positions):
        self._motion.clear_waypoints()
        self._motion.set_joint_names(positions.keys())
        waypoint = MotionWaypoint()
        waypoint.set_joint_angles(positions.values())
        self._motion.append_waypoint(waypoint)
        return self._motion.send_trajectory()