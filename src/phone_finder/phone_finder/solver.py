import numpy as np
from scipy.optimize import minimize
from collections import OrderedDict
import copy
n_dim = 3

# 用优化方法寻找手机位置
def minimize_dist_error(
    sample_pos: np.ndarray, # 采样点位置，with shape [n_samples, n_dim]
    dist: np.ndarray, # 在对应采样点出采样到的与手机之间的距离，with shape [n_targets, n_samples]
    weight: np.ndarray = None # with shape [n_samples, ]
    ):
    n_targets = dist.shape[0]
    # 用最接近手机的采样点坐标初始化手机坐标
    target_pos = sample_pos[dist.argmin(axis=1)].copy()
    # 要最小化的目标函数：当前假设的手机位置到各个采样点之间的距离与真实距离的差的平方和
    def objective_func(target_pos, target_index):
        # 计算当前假定的手机位置与各个采样点之间的距离
        cur_dist = np.sqrt(((sample_pos - target_pos[None, :])**2).sum(axis=-1))
        # 计算距离误差
        error = (cur_dist - dist[target_index]) ** 2
        # TODO：对距离误差进行加权。
        # 想法：手机和采样点之间的距离越短，干扰或者噪声应该越小（我猜的）
        # 这样采样到的距离就应该越可信，最终计算出来的手机位置与采样点之间的距离
        # 也应该和这个采样到的距离差得越小。
        if weight is not None:
            error *= weight
        return float(error.sum())

    results, success = [], []
    # 遍历所有手机
    for i in range(n_targets):
        obj_func = lambda x: objective_func(x, i)
        # 寻找能使目标函数取得最小值的手机位置
        result = minimize(obj_func, target_pos[i], method='Nelder-Mead')
        results.append(result.x)
        success.append(result.success)
    return results, success


class Buffer:
    """一个类，用来管理收集到的数据
    """
    def __init__(self, buffer_size, copy=True):
        self.copy = copy
        self.buffer_size = buffer_size
        self.buffer = []
    def __call__(self, x=None):
        assert not self.is_empty() or x is not None
        if x is not None:
            if self.copy:
                self.buffer.append(copy.deepcopy(x))
            else:
                self.buffer.append(x)
            while len(self.buffer) > self.buffer_size:
                self.buffer.pop(0)
        return self.buffer[-1]
    def empty(self):
        while not self.is_empty():
            self.buffer.pop(0)
    def is_empty(self):
        return len(self.buffer) == 0
    def apply(self, func, *args, **kwargs):
        return func(self.buffer, *args, **kwargs)

class Locator:
    """
    定位模块
    """
    def __init__(self, buffer_size):
        self.buffer_size = buffer_size
        self.drone_pos = Buffer(buffer_size)
        self.IDs = []
        self.dist = OrderedDict()
        self.coord = OrderedDict()
        self.found = {}

    def collect(self, drone_pos, IDs, dists):
        """
        记录采样点位置、待定位物的id和采样到的距离。
        """
        self.drone_pos(drone_pos)
        for id, dist in zip(IDs, dists):
            if id not in self.IDs:
                self.IDs.append(id)
                self.dist[id] = Buffer(self.buffer_size)
                self.coord[id] = MovingAverage(buffer_size=self.buffer_size)
                self.found[id] = False
            if not self.found[id]:
                self.dist[id](dist)
    
    def locate(self):
        """
        利用当前已知的距离信息解算待定位物的坐标。
        """
        success_dict = {}
        new_found = []
        drone_pos = self.drone_pos.apply(np.vstack)
        # 遍历所有id，计算坐标，如果计算成功就更新其坐标。
        for id in self.IDs:
            if not self.found[id] and len(self.dist[id].buffer) > self.buffer_size // 2:
                dist = self.dist[id].apply(np.array)[None, ...]
                coord, success = minimize_dist_error(drone_pos, dist)
                coord[0][-1] = min(coord[0][-1], -coord[0][-1])
                success_dict[id] = success[0]
                if success[0]:
                    self.coord[id](coord[0])
                    if self.convergence(id):
                        self.found[id] = True
                        new_found.append(id)

        return success_dict, new_found
    
    def convergence(self, id, length_threshold=None, std_threshold=0.05):
        """
        判断某个手机的坐标是否收敛
        """
        length_threshold = self.buffer_size // 2 if length_threshold is None else length_threshold
        if len(self.coord[id].buffer.buffer) < length_threshold:
            return False
        coords = self.coord[id].buffer.apply(np.stack, axis=0)
        normed_coords = coords / coords.mean(axis=0)
        normed_std = normed_coords.std(axis=0)[:2].mean()
        return normed_std < std_threshold
    
    def get_coord(self, IDs=None):
        """
        根据id查询待定位物的坐标。
        """
        if IDs is None or isinstance(IDs, list):
            coords = {}
            id_list = self.IDs if IDs is None else IDs
            for id in id_list:
                if self.coord[id].data is not None:
                    coords[id] = self.coord[id].mean()
            return coords
        elif id in self.IDs:
            return self.coord[id].mean()

class VirtualPhone:
    """
    用来仿真的手机类，在调用的时候输入采样点的位置，输出加了噪声的距离
    实际上前提是已经可以从信号强度（或者时间啊什么的）计算出二者之间的距离
    """
    def __init__(self, id, x, y, z):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
    def __call__(self, coord):
        x, y, z = coord
        dist = ((self.x-x)**2 + (self.y-y)**2 + (self.z-z)**2) ** 0.5
        # 加点噪声，噪声方差与距离有关，距离越大，噪声越明显，也就鼓励飞机往距离近噪声小的地方飞
        dist += np.random.normal(loc=0, scale=0.01*dist)
        return float(dist)

class MovingAverage:
    """
    指数加权移动平均
    """
    def __init__(self, step_size=0.1, buffer_size=10):
        self.data = None
        self.step_size = step_size
        assert step_size < 1. and step_size > 0.
        self.buffer = Buffer(buffer_size)
    def __call__(self, x=None):
        assert x is not None or self.data is not None
        if x is not None:
            self.buffer(x)
            if self.data is None:
                self.data = self.buffer()
            else:
                self.data = self.data * (1. - self.step_size) + self.step_size * self.buffer()
        return self.data
    def mean(self):
        if not self.buffer.is_empty():
            data = self.buffer.apply(np.stack, axis=0)
            return data.mean(axis=0)
    def std(self):
        if not self.buffer.is_empty():
            data = self.buffer.apply(np.stack, axis=0)
            return data.std(axis=0).mean()
    
if __name__ == '__main__':
    from time import time
    n_dim = 2
    sample_pos = np.array([[0., 0.],
                           [1., 1.]])
    dist = np.array([[1., 1.],
                     [2., 2.]])
    tic = time()
    print(minimize_dist_error(sample_pos, dist))
    print(time() - tic)

    a = MovingAverage()
    a(np.array([1, 2, 3]))
    a(np.array([4, 5, 6]))
    print(a.std())