import geometry_msgs

def pose_to_list(pose):
    """
    Convert a Pose or PoseStamped in Python list ((position), (quaternion))
    :param pose: geometry_msgs.msg.PoseStamped or geometry_msgs.msg.Pose
    :return: the equivalent in list ((position), (quaternion))
    """
    if type(pose) == geometry_msgs.msg.PoseStamped:
        return [[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
                [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]]
    elif type(pose) == geometry_msgs.msg.Pose:
        return [[pose.position.x, pose.position.y, pose.position.z],
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]]
    else:
        raise Exception("pose_to_list: parameter of type %s unexpected", str(type(pose)))


def list_to_pose(poselist):
    """
    Convert a pose in the form of a list in PoseStamped
    :param poselist: a pose on the form [[x, y, z], [x, y, z, w]]
    :return: the converted geometry_msgs/Pose object
    """
    p = geometry_msgs.msg.Pose()
    p.position.x = poselist[0][0]
    p.position.y = poselist[0][1]
    p.position.z = poselist[0][2]
    p.orientation.x = poselist[1][0]
    p.orientation.y = poselist[1][1]
    p.orientation.z = poselist[1][2]
    p.orientation.w = poselist[1][3]
    return p