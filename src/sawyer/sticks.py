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
        self.stick_fk = {"hope" : [], "fear": [], "joints": []}

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

    # Calibration part 3
    def compute_sticks(self):
        """
        Internally compute all sticks according to default layout
        """
        # Starting from lower left (drawing direction)
        self.sticks = {"hope": [], "fear": []}
        y_interval = 0.01      # Interval of sticks in y direction
        z_interval = 0.02      # Interval of sticks in z direction
        group_interval = 0.01  # Interval between each group in y direction
        num_groups = 8         # Number of stick groups per line
        num_sticks = 4         # Number of sticks per group
        height_stick = 0.03    # Length in z direction
        num_height = 8         # Number of sticks in z direction (in height)
        num_points_cart = 20   # Number of cartesian points for the actual drawing motion

        emotion = "fear"
        z_stick = self.stick_T_world[0][2]
        col = 0
        for h in range(num_height):
            y_stick = self.stick_T_world[0][1]
            for g in range(num_groups):
                for i in range(num_sticks):
                    # Add the end stick
                    rotation = self.stick_T_world[1]
                    position = [self.stick_T_world[0][0], y_stick, z_stick + height_stick]
                    end_stick = [position, rotation]
                    #self.tfb.sendTransform(end_stick[0], end_stick[1], rospy.Time.now(), str(col), "world")
                    motion = [end_stick]

                    # Add the cartesian motion
                    for p in range(num_points_cart):
                        motion_point = deepcopy(end_stick)
                        motion_point[0][2] = z_stick + height_stick - height_stick * (p+1) / num_points_cart
                        motion.append(motion_point)
                        #if i == 0 and g == 0:
                        #    tfb.sendTransform(motion_point, [0, 0, 0, 1], rospy.Time.now(), str(col)+'-'+str(p), "world")
                    
                    self.sticks[emotion].append(motion)
                    y_stick += y_interval
                    col += 1
                y_stick += group_interval
            z_stick += z_interval + height_stick
            if len(self.sticks["fear"]) == num_groups*num_height*num_sticks/2:
                emotion = "hope"
                z_stick += 0.02
        rospy.loginfo(str(len(self.sticks["hope"])) + " + " + str(len(self.sticks["fear"])) + " sticks generated")

    # Calibration part 4
    def compute_cartesian_motions(self):
        rospy.loginfo("Computing all cartesian motions...")
        self.stick_fk = {"hope" : [], "fear": [], "joints": []}
        seed = None
        for emotion in ["fear", "hope"]:
            for stick_id, stick in enumerate(self.sticks[emotion]):
                motion = []  # The cartesian motion
                for point_id, point in enumerate(stick):
                    hand_T_world = transformations.multiply_transform(point, self.hand_T_stick)
                    self.tfb.sendTransform(hand_T_world[0], hand_T_world[1], rospy.Time.now(), "eik", "world")
                    #rospy.sleep(0.01)
                    result = self.limb.ik_request(transformations.list_to_pose(hand_T_world), joint_seed=seed)
                    if result == False:
                        rospy.logerr("IK can't reach point for stick {} ({})".format(stick_id, emotion))
                        return False
                    else:
                        motion.append(result.values())
                        seed = result
                self.stick_fk[emotion].append(motion)
                self.stick_fk["joints"] = result.keys()
        rospy.loginfo("Cartesian motions complete")
        return True

    # Calibration part 5
    def overwrite_cartesian_motions(self):
        with open(join(self.rospack.get_path("cs_sawyer"), "config/motions.json"), "w") as f:
            json.dump(self.stick_fk, f)

    def get_cartesian_motions(self):
        with open(join(self.rospack.get_path("cs_sawyer"), "config/motions.json")) as f:
            self.stick_fk = json.load(f)
        return self.stick_fk