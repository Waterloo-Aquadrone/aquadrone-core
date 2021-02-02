import numpy as np


class Idx:
    NUM = 13
    # 3 position, 4 orientation (quaternion), 3 velocity, 3 angular velocity
    x, y, z, Ow, Ox, Oy, Oz, Vx, Vy, Vz, Wx, Wy, Wz = np.arange(NUM)
