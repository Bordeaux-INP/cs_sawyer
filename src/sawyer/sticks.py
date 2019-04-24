import rospy
import json
import tf
import rospkg
from copy import deepcopy
from sawyer import transformations
from os.path import join
from geometry_msgs.msg import PoseStamped


class Sticks(object):
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.tfl = tf.TransformListener(interpolate=False, cache_time=rospy.Duration(1000))
        self.tfb = tf.TransformBroadcaster()
        self.stick_T_hand = [[0, 0, 0], [0, 0, 0, 1]]
        self.hand_T_stick = [[0, 0, 0], [0, 0, 0, 1]]
        self.stick_T_world = [[0, 0, 0], [0, 0, 0, 1]]
        self.sticks = {"hope": [], "fear": []}

    def set_limb(self, limb):
        self.limb = limb

    def calibrate(self, z_offset):
        # The calibration part that can't fail
        self.set_current_as_origin()
        self.set_pen_offset(z_offset)
        self.compute_sticks()
        # The part that might fail
        return self.compute_cartesian_motions()

    # Calibration part 1
    def set_current_as_origin(self):
        """
        Sets the current limb pose as the origin
        Be careful to position the limb prior to calling this function
        """
        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x = self.limb.endpoint_pose()["position"].x
        ps.pose.position.y = self.limb.endpoint_pose()["position"].y
        ps.pose.position.z = self.limb.endpoint_pose()["position"].z
        ps.pose.orientation.z = 1
        self.stick_T_hand = transformations.pose_to_list(self.tfl.transformPose("right_hand", ps))

    # Call with z_offset = 0.19
    # Calibration part 2
    def set_pen_offset(self, z_offset):
        """
        Precises the transformation between the right_hand and the pen
        """
        self.stick_T_hand[0][2] += z_offset

        # Stick board is perpendical to the world, multiply by conjuguate
        self.stick_T_hand[1] = tf.transformations.quaternion_conjugate(self.stick_T_hand[1])

        ps = PoseStamped()
        ps.header.stamp = rospy.Time(0)
        ps.header.frame_id = "right_hand"
        ps.pose = transformations.list_to_pose(self.stick_T_hand)
        self.stick_T_world = transformations.pose_to_list(self.tfl.transformPose("world", ps))
        self.hand_T_stick = transformations.inverse_transform(self.stick_T_hand)

    def get_drawing_cartesian_motion_vertical(self, end_stick, x_approach_dist, z_stick, height_stick, emotion):
        # Stick approach
        approach = deepcopy(end_stick)
        approach[0][0] += x_approach_dist
        approach[0][2] = z_stick + height_stick

        # Stick init
        init = deepcopy(end_stick)
        init[0][2] = z_stick + height_stick
        
        # Stick drawing
        drawing = deepcopy(end_stick)
        drawing[0][2] = z_stick

        # Stick retreat
        retreat = deepcopy(end_stick)
        retreat[0][0] += x_approach_dist
        retreat[0][2] = z_stick

        motion = {"approach": {"type":"joint", "pose": transformations.multiply_transform(approach, self.hand_T_stick)},
                    "init": {"type":"cart", "pose": transformations.multiply_transform(init, self.hand_T_stick)},
                    "drawing": {"type":"cart", "pose": transformations.multiply_transform(drawing, self.hand_T_stick)},
                    "retreat": {"type":"cart", "pose": transformations.multiply_transform(retreat, self.hand_T_stick)}}

        self.tfb.sendTransform(init[0], init[1], rospy.Time.now(), emotion + str(len(self.sticks[emotion])), "world")
        return motion

    def get_drawing_cartesian_motion_horizontal(self, start_stick_horizontal, x_approach_dist, length_stick, emotion):
        # Stick approach
        approach = deepcopy(start_stick_horizontal)
        approach[0][0] += x_approach_dist

        # Stick init
        init = deepcopy(start_stick_horizontal)
        
        # Stick drawing
        drawing = deepcopy(start_stick_horizontal)
        drawing[0][1] -= length_stick

        # Stick retreat
        retreat = deepcopy(start_stick_horizontal)
        retreat[0][1] -= length_stick
        retreat[0][0] += x_approach_dist

        motion = {"approach": {"type":"joint", "pose": transformations.multiply_transform(approach, self.hand_T_stick)},
                    "init": {"type":"cart", "pose": transformations.multiply_transform(init, self.hand_T_stick)},
                    "drawing": {"type":"cart", "pose": transformations.multiply_transform(drawing, self.hand_T_stick)},
                    "retreat": {"type":"cart", "pose": transformations.multiply_transform(retreat, self.hand_T_stick)}}

        self.tfb.sendTransform(init[0], init[1], rospy.Time.now(), emotion + str(len(self.sticks[emotion])), "world")
        return motion

    # Calibration part 3
    def compute_sticks(self):
        """
        Internally compute all sticks according to default layout
        """
        # Starting from lower left (drawing direction)
        self.sticks = {"hope": [], "fear": []}
        y_interval = 0.01      # Interval of sticks in y direction
        z_interval = 0.01      # Interval of sticks in z direction
        group_interval = 0.01  # Interval between each group in y direction
        num_groups = 10        # Number of stick groups per line
        num_sticks = 4         # Number of sticks per group
        height_stick = 0.02    # Length in z direction
        num_height = 7         # Number of sticks in z direction (in height)
        x_approach_dist = 0.05 # Distance from the board for the approach
        horiz_y_start = 0.005

        emotion = "fear"
        z_stick = self.stick_T_world[0][2]
        col = 0
        for e in ["fear", "hope"]:
            for h in range(num_height):
                y_stick = self.stick_T_world[0][1]
                for g in range(num_groups):
                    for i in range(num_sticks):
                        # This is the end side of the stick
                        rotation = self.stick_T_world[1]
                        position = [self.stick_T_world[0][0], y_stick, z_stick + height_stick]

                        motion = self.get_drawing_cartesian_motion_vertical([position, rotation],
                                                                            x_approach_dist, z_stick,
                                                                            height_stick, e)                      
                        self.sticks[emotion].append(motion)
                        y_stick += y_interval
                        col += 1

                    # Horizontal line
                    start_position_horiz = [[position[0], position[1] + horiz_y_start, z_stick + height_stick/2], rotation]
                    length_stick = 2*horiz_y_start + num_sticks*y_interval
                    motion_horiz = self.get_drawing_cartesian_motion_horizontal(start_position_horiz, x_approach_dist,
                                                                                length_stick, e)
                    self.sticks[emotion].append(motion_horiz)

                    y_stick += group_interval

                z_stick += z_interval + height_stick
                if len(self.sticks["fear"]) == num_groups*num_height*(num_sticks+1) and len(self.sticks["hope"]) == 0:
                    emotion = "hope"
                    z_stick += 0.05
        self.sticks["dimensions"] = (num_height, num_groups, num_sticks + 1)  # + 1 means that there is also the horizontal line
        rospy.loginfo(str(len(self.sticks["hope"])) + " + " + str(len(self.sticks["fear"])) + " sticks generated")

    # Calibration part 4
    def compute_cartesian_motions(self):
        rospy.loginfo("Computing all cartesian motions...")
        seed = None
        for emotion in ["fear", "hope"]:
            for stick_id, stick in enumerate(self.sticks[emotion]):
                for mtype, point in stick.items():
                    #rospy.sleep(1)
                    result = self.limb.ik_request(transformations.list_to_pose(point["pose"]), joint_seed=seed)
                    if result == False:
                        rospy.logerr("IK can't reach point for stick {} ({})".format(stick_id, emotion))
                        return False
                    else:
                        self.sticks["joints"] = result.keys()
                        self.sticks[emotion][stick_id][mtype]["joints"] = result.values()
                        seed = result
                #rospy.sleep(1)
        rospy.loginfo("Cartesian motions complete")
        return True

    # Calibration part 5
    def overwrite_cartesian_motions(self):
        with open(join(self.rospack.get_path("cs_sawyer"), "config/sticks.json"), "w") as f:
            json.dump(self.sticks, f)

    def get_cartesian_motions(self):
        try:
            with open(join(self.rospack.get_path("cs_sawyer"), "config/sticks.json")) as f:
                self.sticks = json.load(f)
        except IOError:
            self._sticks = {"hope": [], "fear": []}
        finally:
            return self.sticks