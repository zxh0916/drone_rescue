from collections import OrderedDict
import numpy as np
a = b = np.arange(9).reshape(3, 3)
print(np.stack([a, b], axis=0).shape)
c = d = np.array(3)
print(np.stack([c, d], axis=0).shape)
e = OrderedDict()
e['0'] = 0
e['1'] = 1
print(np.array(list(e.values())), '1' in e.keys())
f = float('inf')
print(f == float('inf'))
g = np.arange(99)
print(g.std() / g.mean(), (g*2).std() / (g*2).mean())
h = np.array([[1, 2, 3]])
print(h.std(axis=0))
i = np.empty((3, 3))
i.fill(float('inf'))
print(i)
j = np.arange(9).reshape(3, 3)
print((j/j.mean(axis=0)).std(axis=0), ((j*10)/(j*10).mean(axis=0)).std(axis=0))