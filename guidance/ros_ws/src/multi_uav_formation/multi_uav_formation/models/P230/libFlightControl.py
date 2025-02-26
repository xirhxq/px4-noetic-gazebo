#!/usr/bin/env python3

from models.M300.DjiUtils import *

PI = math.acos(-1)
DEG2RAD_COE = PI / 180
RAD2DEG_COE = 180 / PI

RESET = "\033[0m"
BLACK = "\033[30m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"
BOLDBLACK = "\033[1m\033[30m"
BOLDRED = "\033[1m\033[31m"
BOLDGREEN = "\033[1m\033[32m"
BOLDYELLOW = "\033[1m\033[33m"
BOLDBLUE = "\033[1m\033[34m"
BOLDMAGENTA = "\033[1m\033[35m"
BOLDCYAN = "\033[1m\033[36m"
BOLDWHITE = "\033[1m\033[37m"


class DataStat:
    def __init__(self):
        self.cnt = 0
        self.mean = 0.0
        self.std = 0.0
        self.rms = 0.0

    def new_data(self, x):
        self.cnt += 1
        self.std = math.sqrt(
            (self.std ** 2 / self.cnt * (self.cnt - 1) + (x - self.mean) ** 2 / self.cnt * (self.cnt - 1) / self.cnt))
        self.mean = self.mean / self.cnt * (self.cnt - 1) + x / self.cnt
        self.rms = math.sqrt((self.rms ** 2 / self.cnt * (self.cnt - 1) + x ** 2 / self.cnt))


class MedianFilter:
    def __init__(self, size):
        self.size = size
        self.v = []

    def new_data(self, nd):
        self.v.append(nd)
        while len(self.v) > self.size:
            self.v.pop(0)

    def result(self):
        tmp = sorted(self.v)
        return (tmp[(len(tmp) - 1) // 2] + tmp[len(tmp) // 2]) / 2

    def output(self):
        print("Now Median Filter Contains:", end="")
        for i in self.v:
            print(f"\t{i}", end="")
        print()


class AverageFilter:
    def __init__(self, size):
        self.size = size
        self.v = []

    def new_data(self, nd):
        self.v.append(nd)
        while len(self.v) > self.size:
            self.v.pop(0)

    def result(self):
        return sum(self.v) / len(self.v)

    def output(self):
        print("Now Average Filter Contains:", end="")
        for i in self.v:
            print(f"\t{i}", end="")
        print()


class XYZMedianFilter:
    def __init__(self, size=11):
        self.x = MedianFilter(size)
        self.y = MedianFilter(size)
        self.z = MedianFilter(size)

    def new_data(self, nd):
        self.x.new_data(nd.x)
        self.y.new_data(nd.y)
        self.z.new_data(nd.z)

    def result(self):
        res = Point()  # create an empty object
        res.x = self.x.result()
        res.y = self.y.result()
        res.z = self.z.result()
        return res

    def output(self):
        self.x.output()
        self.y.output()
        self.z.output()


class XYZAverageFilter:
    def __init__(self, size=11):
        self.x = AverageFilter(size)
        self.y = AverageFilter(size)
        self.z = AverageFilter(size)

    def new_data(self, nd):
        self.x.new_data(nd.x)
        self.y.new_data(nd.y)
        self.z.new_data(nd.z)

    def result(self):
        res = Point()  # create an empty object
        res.x = self.x.result()
        res.y = self.y.result()
        res.z = self.z.result()
        return res

    def output(self):
        self.x.output()
        self.y.output()
        self.z.output()


def quaternion2euler(quat):
    angle = [0.0, 0.0, 0.0]
    angle[0] = math.atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                          1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]))
    angle[1] = math.asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]))
    angle[2] = math.atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]),
                          -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]))
    return angle


def euler2dcm(euler):
    phi = euler[0]
    theta = euler[1]
    psi = euler[2]
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    sin_psi = math.sin(psi)
    cos_psi = math.cos(psi)

    dcm = np.zeros((3, 3))
    dcm[0][0] = cos_theta * cos_psi
    dcm[0][1] = cos_theta * sin_psi
    dcm[0][2] = -sin_theta

    dcm[1][0] = -cos_phi * sin_psi + sin_phi * sin_theta * cos_psi
    dcm[1][1] = cos_phi * cos_psi + sin_phi * sin_theta * sin_psi
    dcm[1][2] = sin_phi * cos_theta

    dcm[2][0] = sin_phi * sin_psi + cos_phi * sin_theta * cos_psi
    dcm[2][1] = -sin_phi * cos_psi + cos_phi * sin_theta * sin_psi
    dcm[2][2] = cos_phi * cos_theta

    return dcm


def dcm_transpose(dcm):
    return np.transpose(dcm)


def matrix_multiply(axis, dcm):
    return np.dot(dcm, axis)


def limit_value(x, sat):
    return max(-abs(sat), min(abs(sat), x))


def angle_transf(euler, bias, pixel):
    q = [0.0, 0.0, 0.0]
    theta = euler[1] + bias
    M1 = -math.cos(theta) * math.sin(euler[2]) * math.cos(pixel[1]) * math.cos(pixel[0]) + (math.sin(theta) * math.sin(euler[2]) * math.cos(euler[0]) + math.cos(euler[2]) * math.sin(euler[0])) * math.sin(pixel[1]) - (-math.sin(theta) * math.sin(euler[2]) * math.sin(euler[0]) + math.cos(euler[2]) * math.cos(euler[0])) * math.cos(pixel[1]) * math.sin(pixel[0])
    N1 = math.cos(theta) * math.cos(euler[2]) * math.cos(pixel[1]) * math.cos(pixel[0]) + (-math.sin(theta) * math.cos(euler[2]) * math.cos(euler[0]) + math.sin(euler[2]) * math.sin(euler[0])) * math.sin(pixel[1]) - (math.sin(theta) * math.cos(euler[2]) * math.sin(euler[0]) + math.sin(euler[2]) * math.cos(euler[0])) * math.cos(pixel[1]) * math.sin(pixel[0])

    q[1] = math.asin(math.sin(theta) * math.cos(pixel[1]) * math.cos(pixel[0]) + math.cos(theta) * math.cos(euler[0]) * math.sin(pixel[1]) + math.cos(theta) * math.sin(euler[0]) * math.cos(pixel[1]) * math.sin(pixel[0]))
    q[2] = math.atan2(-M1, N1)
    return q


def degree_round(deg):
    if deg < -180.0:
        return deg + 360.0
    if deg > 180.0:
        return deg - 360.0
    return deg


def degree_round_0_to_360(deg):
    while deg < 0.0:
        deg += 360.0
    while deg >= 360.0:
        deg -= 360.0
    return deg


def rad_round(rad):
    if rad < -PI:
        return rad + 2 * PI
    if rad > PI:
        return rad - 2 * PI
    return rad


def nearly_is(a, b, tol=0.1):
    return abs(a - b) <= tol


def e2b(a, RE2b):
    xx, yy, zz = a.x, a.y, a.z
    a.x = RE2b[0][0] * xx + RE2b[0][1] * yy + RE2b[0][2] * zz
    a.y = RE2b[1][0] * xx + RE2b[1][1] * yy + RE2b[1][2] * zz
    a.z = RE2b[2][0] * xx + RE2b[2][1] * yy + RE2b[2][2] * zz


def b2e(a, RE2b):
    xx, yy, zz = a.x, a.y, a.z
    a.x = RE2b[0][0] * xx + RE2b[1][0] * yy + RE2b[2][0] * zz
    a.y = RE2b[0][1] * xx + RE2b[1][1] * yy + RE2b[2][1] * zz
    a.z = RE2b[0][2] * xx + RE2b[1][2] * yy + RE2b[2][2] * zz


def set_value(a, b):
    a.x = b.x
    a.y = b.y
    a.z = b.z


def set_value_from_array(a, b):
    a.x = b[0]
    a.y = b[1]
    a.z = b[2]


def set_value_to_array(a, b):
    a[0] = b.x
    a[1] = b.y
    a[2] = b.z


def set_value_xyz(a, x, y, z):
    a.x = x
    a.y = y
    a.z = z


def set_value_quaternion(a, b):
    a.x = b.x
    a.y = b.y
    a.z = b.z
    a.w = b.w


def saturate_vel(a, b):
    a.x = limit_value(a.x, b.x)
    a.y = limit_value(a.y, b.y)
    a.z = limit_value(a.z, b.z)


def new_point(x, y, z):
    p = Point()
    set_value_xyz(p, x, y, z)
    return p


def new_point_from_array(p):
    res = Point()
    set_value_from_array(res, p)
    return res


def dis(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def dissq(a, b):
    return (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2


def dis2d(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def angle2d(a, b):
    return math.atan2(b.y - a.y, b.x - a.x)


def minus(a, b):
    res = Point()
    res.x = a.x - b.x
    res.y = a.y - b.y
    res.z = a.z - b.z
    return res


def plus(a, b):
    res = Point()
    res.x = a.x + b.x
    res.y = a.y + b.y
    res.z = a.z + b.z
    return res


def scale(a, b):
    res = Point()
    res.x = a.x * b
    res.y = a.y * b
    res.z = a.z * b
    return res


def interpolate(a, b, ratio):
    res = Point()
    res.x = a.x + (b.x - a.x) * ratio
    res.y = a.y + (b.y - a.y) * ratio
    res.z = a.z + (b.z - a.z) * ratio
    return res


def norm(a):
    return math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)


def output_str(a):
    return f'({a.x:.2f}, {a.y:.2f}, {a.z:.2f})'


def put_discrete_points(v, b, n):
    assert len(v) > 0
    a = v[-1]
    for i in range(1, n + 1):
        v.append(plus(a, scale(minus(b, a), 1.0 * i / n)))


def output_vector(v):
    for i in v:
        print(output_str(i))


def encode_uint8(x, n):
    return (x >> (8 * n)) & ((1 << 8) - 1)


def decode_uint8(v, pos):
    return (v[pos] << 16) + (v[pos + 1] << 8) + v[pos + 2]


def generate_smooth_path(start, end, num_points):
    path_points = []
    step_x = (end.x - start.x) / num_points
    step_y = (end.y - start.y) / num_points
    step_z = (end.z - start.z) / num_points
    for i in range(num_points):
        intermediate_point = new_point(start.x + step_x * (i + 1), start.y + step_y * (i + 1), start.z + step_z * (i + 1))
        path_points.append(intermediate_point)

    return path_points


def constant_acc_to_target_point(start_point, end_point, now_point):
    acc = 0.5
    path_len = norm(minus(start_point, end_point))
    vel = 0
    target_vel = Point()
    if norm(minus(now_point, start_point)) < 100:
        vel = math.sqrt(norm(minus(now_point, start_point)) * 2 * acc)
    if 100 < norm(minus(now_point, start_point)) < (path_len - 100):
        vel = 10
    if norm(minus(now_point, start_point)) > (path_len - 100):
        vel = math.sqrt(norm(minus(now_point, end_point)) * 2 * acc)

    target_vel.x = vel * (minus(end_point, now_point).x) / norm(minus(end_point, now_point))
    target_vel.y = vel * (minus(end_point, now_point).y) / norm(minus(end_point, now_point))
    target_vel.z = vel * (minus(end_point, now_point).z) / norm(minus(end_point, now_point))

    print(f"target_vel.x: {target_vel.x:.2f}, target_vel.y: {target_vel.y:.2f}, target_vel.z: {target_vel.z:.2f}")
    return target_vel