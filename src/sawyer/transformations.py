import geometry_msgs
from tf import transformations
from numpy import ndarray, dot, sqrt, array, arccos, inner, zeros, fill_diagonal

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

def list_to_pose_stamped(poselist, stamp=None, frame_id="base"):
    ps = geometry_msgs.msg.PoseStamped()
    if stamp is not None:
        ps.header.stamp = stamp
    ps.header.frame_id = frame_id
    ps.pose = list_to_pose(poselist)
    return ps

def quat_rotate(rotation, vector):
    """
    Rotate a vector according to a quaternion. Equivalent to the C++ method tf::quatRotate
    :param rotation: the rotation
    :param vector: the vector to rotate
    :return: the rotated vector
    """
    def quat_mult_point(q, w):
        return (q[3] * w[0] + q[1] * w[2] - q[2] * w[1],
                q[3] * w[1] + q[2] * w[0] - q[0] * w[2],
                q[3] * w[2] + q[0] * w[1] - q[1] * w[0],
                -q[0] * w[0] - q[1] * w[1] - q[2] * w[2])

    q = quat_mult_point(rotation, vector)
    q = transformations.quaternion_multiply(q, transformations.quaternion_inverse(rotation))
    return [q[0], q[1], q[2]]


def _is_indexable(var):
    try:
        var[0]
    except TypeError:
        return False
    except IndexError:
        return True
    else:
        return True

def multiply_transform(t1, t2):
    """
    Combines two transformations together
    The order is translation first, rotation then
    :param t1: [[x, y, z], [x, y, z, w]] or matrix 4x4
    :param t2: [[x, y, z], [x, y, z, w]] or matrix 4x4
    :return: The combination t1-t2 in the form [[x, y, z], [x, y, z, w]] or matrix 4x4
    """
    if _is_indexable(t1) and len(t1) == 2:
        return [list(quat_rotate(t1[1], t2[0]) + array(t1[0])),
                list(transformations.quaternion_multiply(t1[1], t2[1]))]
    else:
        return dot(t1, t2)

def inverse_transform(t):
    """
    Return the inverse transformation of t
    :param t: A transform [[x, y, z], [x, y, z, w]]
    :return: t2 such as multiply_transform_(t, t2) = [[0, 0, 0], [0, 0, 0, 1]]
    """
    return [quat_rotate(transformations.quaternion_inverse(t[1]), [-t[0][0], -t[0][1], -t[0][2]]), transformations.quaternion_inverse(t[1])]
