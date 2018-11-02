import numpy as np
import matplotlib.pyplot as plt
import transforms3d.euler as euler

# find nearest element in a array return the index
def find_nearest(array,value):
    idx = np.abs(array-value).argmin()
    return idx

def warp2pi(x):
    while x > np.pi:
        x = x - 2*np.pi
    while x < -np.pi:
        x = x + 2*np.pi
    return x

path1 = np.genfromtxt('result_vins.csv', delimiter=',',names=['timestamp','x', 'y', 'z','qw','qx','qy','qz'])
path2 = np.genfromtxt('result_vive.csv', delimiter=',',names=['timestamp','x', 'y', 'z','qw','qx','qy','qz'])


ground_truth = path2
ground_truth_size = ground_truth['timestamp'].size

time = (ground_truth['timestamp'] - ground_truth['timestamp'][0])

# print(time)

course = np.zeros((1,ground_truth_size)) # length of the path

for i in range(ground_truth_size):
    d = 0
    for j in range(0,i):
        p1 = np.array([ground_truth['x'][j],ground_truth['y'][j],ground_truth['z'][j]])
        p2 = np.array([ground_truth['x'][j+1],ground_truth['y'][j+1],ground_truth['z'][j+1]])
        d = d + np.linalg.norm(p1-p2)
    course[0][i] = d

# data alignment according to they're timestamp
align_idx = np.zeros((2,ground_truth_size),dtype=np.int64)
for i in range(0,ground_truth_size):
    align_idx[0][i] = find_nearest(path1['timestamp'],ground_truth['timestamp'][i])


# show path
plt.figure(1)
plt.plot(path1['x'],path1['y'], label='VINS') # path 1
plt.plot(ground_truth['x'],ground_truth['y'], label='VIVE') # ground truth
plt.xlabel('x [m]') 
plt.ylabel('y [m]')
plt.axis('equal')
plt.grid()
plt.legend(loc='best', shadow=False)
plt.title('Path')

# calculate position error
error1 = np.zeros((3,ground_truth_size))

for i in range(0,ground_truth_size):
    idx = align_idx[0][i]
    error1[0][i] = ground_truth['x'][i]-path1['x'][idx]
    error1[1][i] = ground_truth['y'][i]-path1['y'][idx]
    error1[2][i] = ground_truth['z'][i]-path1['z'][idx]


plt.figure(2)
plt.subplot(311)
plt.plot(course[0], error1[0], label='VINS')
# plt.legend(loc='best', shadow=False)
plt.grid()
plt.ylabel('x [m]')
plt.title('Translation Error')

plt.subplot(312)
plt.plot(course[0], error1[1], label='VINS')
plt.grid()
plt.ylabel('y [m]')

plt.subplot(313)
plt.plot(course[0], error1[2], label='VINS')
plt.grid()
plt.ylabel('z [m]')
plt.xlabel('distance [m]')

# calculate accuracy
plt.figure(3)
accuracy1 = np.zeros((1,ground_truth_size))

strat_idx = find_nearest(time,5)

for i in range(strat_idx,ground_truth_size):
    idx = align_idx[0][i]
    p1 = np.array([ground_truth['x'][i],ground_truth['y'][i],ground_truth['z'][i]])
    p2 = np.array([path1['x'][idx],path1['y'][idx],path1['z'][idx]])
    accuracy1[0][i] = 100*np.linalg.norm(p1-p2)/course[0][i]

plt.plot(course[0][strat_idx:-1], accuracy1[0][strat_idx:-1], label='VINS')
plt.xlabel('distance [m]')
plt.ylabel('error rate [%]')
plt.legend(loc='best', shadow=False)
plt.title('Accuracy')
plt.grid(True)

plt.figure(4)
yaw_error1 = np.zeros((1,ground_truth_size))
pitch_error1 = np.zeros((1,ground_truth_size))
roll_error1 = np.zeros((1,ground_truth_size))
for i in range(0,ground_truth_size):
    idx = align_idx[0][i]
    e1 = euler.quat2euler([ground_truth['qw'][i],ground_truth['qx'][i],ground_truth['qy'][i],ground_truth['qz'][i]],axes='rzyx')
    e2 = euler.quat2euler([path1['qw'][idx],path1['qx'][idx],path1['qy'][idx],path1['qz'][idx]],axes='rzyx')
    yaw_error1[0][i] = e1[0] - e2[0]
    pitch_error1[0][i] = e1[1] - e2[1]
    roll_error1[0][i] = e1[2] - e2[2]

    yaw_error1[0][i] = warp2pi(yaw_error1[0][i])
    pitch_error1[0][i] = warp2pi(pitch_error1[0][i])
    roll_error1[0][i] = warp2pi(roll_error1[0][i])

plt.subplot(311)
plt.plot(course[0], yaw_error1[0]*180/np.pi, label='yaw')
plt.ylabel('yaw error [deg]')
plt.title('Rotation Error')
plt.grid(True)
plt.subplot(312)
plt.plot(course[0], pitch_error1[0]*180/np.pi, label='pitch')
plt.ylabel('pitch error [deg]')
plt.grid(True)
plt.subplot(313)
plt.plot(course[0], roll_error1[0]*180/np.pi, label='roll')
plt.ylabel('roll error [deg]')
# plt.ylim(-1.5, 1.5) 
# plt.legend(loc='best', shadow=False)
plt.xlabel('distance [m]')
plt.grid(True)

# plt.figure(4)
# t = np.zeros((2,ground_truth_size),dtype=np.int64)
# for i in range(0,ground_truth_size):
#     idx = align_idx[0][i]
#     t[0][i] = ground_truth['timestamp'][i] - path1['timestamp'][idx]
#     print(ground_truth['timestamp'][i],path1['timestamp'][idx],i,idx)

#     idx = align_idx[1][i]
#     t[1][i] = ground_truth['timestamp'][i] - path2['timestamp'][idx]
    
# print(t[0])
# plt.plot(time,t[0])
# plt.plot(time,t[1])

x_error_mean = np.mean(error1[0])
y_error_mean = np.mean(error1[1])
z_error_mean = np.mean(error1[2])
x_error_std = np.std(error1[0])
y_error_std = np.std(error1[1])
z_error_std = np.std(error1[2])

yaw_error_mean = np.mean(yaw_error1[0])
pitch_error_mean = np.mean(pitch_error1[0])
roll_error_mean = np.mean(roll_error1[0])
yaw_error_std = np.std(yaw_error1[0])
pitch_error_std = np.std(pitch_error1[0])
roll_error_std = np.std(roll_error1[0])

print('======================RESULT=======================')
print('Translational Error:')
print('x error mean:%7.4f(m),  std:%7.4f(m)'% (x_error_mean,x_error_std))
print('y error mean:%7.4f(m),  std:%7.4f(m)'% (y_error_mean,y_error_std))
print('z error mean:%7.4f(m),  std:%7.4f(m)'% (z_error_mean,z_error_std))
print('==================================================')
print('Rotational Error:')
print('yaw   error mean:%7.4f(deg),  std:%7.4f(deg)'% (yaw_error_mean,yaw_error_std))
print('pitch error mean:%7.4f(deg),  std:%7.4f(deg)'% (pitch_error_mean,pitch_error_std))
print('roll  error mean:%7.4f(deg),  std:%7.4f(deg)'% (roll_error_mean,roll_error_std))
print('==================================================')
print('Path length:%7.4fm'%(course[0][-1]))
print('==================================================')

plt.show()
