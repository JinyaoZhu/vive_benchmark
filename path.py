import numpy as np
import matplotlib.pyplot as plt

path1 = np.genfromtxt('result_vins.csv', delimiter=',',names=['timestamp','x', 'y', 'z','qw','qx','qy','qz'])
path2 = np.genfromtxt('result_vive.csv', delimiter=',',names=['timestamp','x', 'y', 'z','qw','qx','qy','qz'])


path1_size = path1['timestamp'].size

time = (path1['timestamp'] - path1['timestamp'][0])/1e9

course = np.zeros((1,path1_size)) # length of the path

for i in range(path1_size):
    d = 0
    for j in range(0,i):
        p1 = np.array([path1['x'][j],path1['y'][j],path1['z'][j]])
        p2 = np.array([path1['x'][j+1],path1['y'][j+1],path1['z'][j+1]])
        d = d + np.linalg.norm(p1-p2)
    course[0][i] = d

# show path
plt.figure(1)
plt.plot(path1['x'],path1['y'], label='vins')
plt.plot(path2['x'],path2['y'], label='vive')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.axis('equal')
plt.grid()
plt.legend(loc='best', shadow=False)
plt.title('PLOT PATH')

print('course:%fm'%(course[0][-1]))

plt.show()
