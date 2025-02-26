import math
import numpy as np

RESET = "\033[0m"
BOLD = "\033[1m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
CLEAR = "\033[2J"


STD_SHAPE = [(3,), (3, 1)]
GRAVITY = 9.81
gravity = 9.81

def ned2frdRotationMatrix(rollAngle, pitchAngle, yawAngle):
    R_z = np.array([
            [np.cos(yawAngle), np.sin(yawAngle), 0],
            [-np.sin(yawAngle), np.cos(yawAngle), 0],
            [0, 0, 1]
        ])

    R_y = np.array([
            [np.cos(pitchAngle), 0, -np.sin(pitchAngle)],
            [0, 1, 0],
            [np.sin(pitchAngle), 0, np.cos(pitchAngle)]
        ])

    R_x = np.array([
            [1, 0, 0],
            [0, np.cos(rollAngle), np.sin(rollAngle)],
            [0, -np.sin(rollAngle), np.cos(rollAngle)]
        ])

    ned2frdRotationMatrix = np.dot(R_x, np.dot(R_y, R_z))
    return ned2frdRotationMatrix

def frd2nedRotationMatrix(rollAngle, pitchAngle, yawAngle):

    R = np.linalg.inv(ned2frdRotationMatrix(rollAngle, pitchAngle, yawAngle))

    return R

def enu2fluRotationMatrix(rollAngle, pitchAngle, yawAngle):
    return ned2frdRotationMatrix(rollAngle, pitchAngle, yawAngle)

def flu2enuRotationMatrix(rollAngle, pitchAngle, yawAngle):
    return frd2nedRotationMatrix(rollAngle, pitchAngle, yawAngle)

def enu2losRotationMatrix(yAxisRotationAngle, zAxisRotationAngle):

    R_y = np.array([
        [np.cos(yAxisRotationAngle), 0, -np.sin(yAxisRotationAngle)],
        [0, 1, 0],
        [np.sin(yAxisRotationAngle), 0, np.cos(yAxisRotationAngle)]
    ])

    R_z = np.array([
        [np.cos(zAxisRotationAngle), np.sin(zAxisRotationAngle), 0],
        [-np.sin(zAxisRotationAngle), np.cos(zAxisRotationAngle), 0],
        [0, 0, 1]
    ])

    R = np.dot(R_y, R_z)

    return R

def rotationMatrix(angleRad, axis):
    """
    Rotation matrix with x/y/z axis doing a right-handed rotation of a certain angle
    :param angleRad: The angle of axis rotation, in rad.
    :param axis: x or y or z.
    :return: The rotation matrix to transfer the coordinate in the frame before rotation to the coordinate in the frame after rotation
    """
    axis = axis.lower()
    assert axis in 'xyz', 'axis must be one of x, y, z'

    if axis.lower() == 'x':
        return np.array([[1, 0, 0],
                         [0, np.cos(angleRad), np.sin(angleRad)],
                         [0, -np.sin(angleRad), np.cos(angleRad)]])
    elif axis.lower() == 'y':
        return np.array([[np.cos(angleRad), 0, -np.sin(angleRad)],
                         [0, 1, 0],
                         [np.sin(angleRad), 0, np.cos(angleRad)]])
    elif axis.lower() == 'z':
        return np.array([[np.cos(angleRad), np.sin(angleRad), 0],
                         [-np.sin(angleRad), np.cos(angleRad), 0],
                         [0, 0, 1]])


def NED2ENU(x):
    return rotationMatrix(np.pi / 2, axis='z') @ rotationMatrix(np.pi, axis='y') @ x

def los2enuRotationMatrix(yAxisRotationAngle, zAxisRotationAngle):
    return np.linalg.inv(enu2losRotationMatrix(yAxisRotationAngle, zAxisRotationAngle))

def makeArray(ls: list):
    assert len(ls) == 3, 'List length should be 3'
    return np.array(ls)


def CHECK(vec: np.ndarray):
    assert vec.shape in STD_SHAPE, f'Shape should be {STD_SHAPE} but here is {vec.shape}'


def ned2enu(vec):
    CHECK(vec)
    T = np.array([
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1]
    ])
    return T @ vec


def enu2ned(vec):
    CHECK(vec)
    T = np.array([
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1]
    ])
    return T @ vec


def quaternion2euler(q):
    [q0, q1, q2, q3] = q
    roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    pitch = np.arcsin(2 * (q0 * q2 - q3 * q1))
    yaw = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
    return np.array([roll, pitch, yaw])


def quaternion2eulerXYZW(q):
    return quaternion2euler([q.w, q.x, q.y, q.z])


def euler2quaternion(attitudeAngle):
    quaternion = [0] * 4
    cosHalfRoll = math.cos(attitudeAngle[0] / 2)
    cosHalfPitch = math.cos(attitudeAngle[1] / 2)
    cosHalfYaw = math.cos(attitudeAngle[2] / 2)
    sinHalfRoll = math.sin(attitudeAngle[0] / 2)
    sinHalfPitch = math.sin(attitudeAngle[1] / 2)
    sinHalfYaw = math.sin(attitudeAngle[2] / 2)

    quaternion[0] = cosHalfRoll * cosHalfPitch * cosHalfYaw + sinHalfRoll * sinHalfPitch * sinHalfYaw
    quaternion[1] = sinHalfRoll * cosHalfPitch * cosHalfYaw - cosHalfRoll * sinHalfPitch * sinHalfYaw
    quaternion[2] = cosHalfRoll * sinHalfPitch * cosHalfYaw + sinHalfRoll * cosHalfPitch * sinHalfYaw
    quaternion[3] = cosHalfRoll * cosHalfPitch * sinHalfYaw - sinHalfRoll * sinHalfPitch * cosHalfYaw
    return quaternion

def wrapToPi(x):
    x = x % (2 * np.pi)
    if x > np.pi:
        x = x - 2 * np.pi
    return x

def rpyENU2NED(rpyRadENU):
    return np.array([rpyRadENU[0], -rpyRadENU[1], yawRadENU2NED(rpyRadENU[2])])


def rpyNED2ENU(rpyRadNED):
    return rpyENU2NED(rpyRadNED)


def yawRadNED2ENU(yawRadNED):
    return (np.pi / 2 - yawRadNED) % (2 * np.pi)


def yawRadENU2NED(yawRadENU):
    return yawRadNED2ENU(yawRadENU)


def arrayString(point):
    ret = '('
    for i in range(len(point)):
        ret += f"{point[i]:.2f}"
        if i != len(point) - 1:
            ret += ', '
    return ret + ')'

def pointString(point):
    assert hasattr(point, 'x') and hasattr(point, 'y') and hasattr(point, 'z'), 'Point should have x, y, z attributes'
    return f"({point.x:.2f}, {point.y:.2f}, {point.z:.2f})"

def velocityString(velociy):
    assert hasattr(velociy, 'vx') and hasattr(velociy, 'vy') and hasattr(velociy, 'vz'), 'Point should have vx, vy, vz attributes'
    return f"({velociy.vx:.2f}, {velociy.vy:.2f}, {velociy.vz:.2f})"

def rpyString(rpyRad):
    assert len(rpyRad) == 3, f'Length of rpy should be 3, but get {len(rpyRad)}'
    rpyDeg = 180 / np.pi * rpyRad
    return f'(roll: {rpyDeg[0]:.2f}, pitch: {rpyDeg[1]:.2f}, yaw: {rpyDeg[2]:.2f})'

def point2Array(point):
    return np.array([point.x, point.y, point.z])

def vector3String(point):
    return f"({point.x:.2f}, {point.y:.2f}, {point.z:.2f})"

def rpyString(rpyRad):
    assert len(rpyRad) == 3, f'Length of rpy should be 3, but get {len(rpyRad)}'
    rpyDeg = 180 / np.pi * rpyRad
    return f'(roll: {rpyDeg[0]:.2f}, pitch: {rpyDeg[1]:.2f}, yaw: {rpyDeg[2]:.2f})'

def accENUYawENU2EulerENUThrust(accENU, yawRadENU, hoverThrottle):
    yawRadNED = yawRadENU2NED(yawRadENU)
    accNED = enu2ned(accENU)
    liftAcc = -(accNED - np.array([0, 0, GRAVITY]))
    print(f"liftAcc = {liftAcc}")
    r1 = math.cos(yawRadNED) * liftAcc[0] + math.sin(yawRadNED) * liftAcc[1]
    r2 = math.sin(yawRadNED) * liftAcc[0] - math.cos(yawRadNED) * liftAcc[1]
    pitchRad = math.atan2(r1, liftAcc[2])
    r3 = math.sin(pitchRad) * r1 + math.cos(pitchRad) * liftAcc[2]
    rollRad = math.atan2(r2, r3)
    controlEulerNED = np.array([rollRad, pitchRad, yawRadNED])

    eulerRadENU = rpyNED2ENU(controlEulerNED)
    print(f"ControlEulerENU = {rpyString(eulerRadENU)}")
    thrust = hoverThrottle * abs(liftAcc[2] / math.cos(rollRad) / math.cos(pitchRad) / GRAVITY)
    return thrust, eulerRadENU

def ENU2NED(x):
    return rotationMatrix(np.pi, axis='y') @ rotationMatrix(-np.pi / 2, axis='z') @ x