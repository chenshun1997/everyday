import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   
import matplotlib.animation as animation              
import numpy as np 
# import rospy
# from geometry_msgs.msg import Twist,Pose,PoseStamped,TwistStamped
# from gazebo_msgs.srv import GetModelState
import sys

# uav_type = 2
# uav_num = 18

# step_time=0.5

# pose_puber=[None]*uav_num
# vel_puber=[None]*uav_num

# plot_x=[0]*(uav_num)
# plot_y=[0]*(uav_num)
# plot_z=[0]*(uav_num)


fig = plt.figure()
plt.ion()
ax = Axes3D(fig)
label_lim = 20

# def scroll_call_back(event):
#     global label_lim
#     if event.button == 'up':
#         label_lim+=2
#         #print('up')
#     elif event.button == 'down':
#         label_lim=label_lim-2 if label_lim>1 else 1
#         #print('down')


# fig.canvas.mpl_connect('scroll_event', scroll_call_back)

def init():
    ax.set_xlim3d(0, label_lim)
    ax.set_ylim3d(label_lim,0)
    ax.set_zlim3d(0, label_lim)

# def pose_sub_callback(msg,id):

#     plot_x[id]=msg.pose.position.x
#     plot_y[id]=msg.pose.position.y
#     plot_z[id]=msg.pose.position.z

# rospy.init_node('visualize')
# rate = rospy.Rate(1/step_time)

# for i in range(uav_num):
#     rospy.Subscriber(uav_type+'_'+str(i)+'/mavros/local_position/pose', PoseStamped, pose_sub_callback,i)  
point = [[0,0,0],[1, 1, 1], [3, 3, 3], [5, 5, 5]]
point1 = [[1, 2, 3], [2, 3, 5], [4, 4, 5],[7,7,7]]
# for i in range(len(point)):
# while True:
    
#     for i in range(len(point)):
#         init()
#         x = point[i][0]
#         y = point[i][1]
#         z = point[i][2]
#         x1 = point1[i][0]
#         y1 = point1[i][1]
#         z1 = point1[i][2]
#         ax.scatter(x, y, z, s=50, marker="^")
#         ax.scatter(x1, y1, z1, s=50, marker="^")
#         # ax.tick_params(axis="x", labelsize=20)
#         # ax.tick_params(axis="y", labelsize=20)
#         # ax.tick_params(axis="z", labelsize=20)
#         plt.pause(1)
#         ax.cla()
while True:
    try:
        for i in range(len(point)):
            init()
            x = point[i][0]
            y = point[i][1]
            z = point[i][2]
            x1 = point1[i][0]
            y1 = point1[i][1]
            z1 = point1[i][2]
            ax.scatter(x, y, z, s=50, marker="o")
            ax.scatter(x1, y1, z1, s=50, marker="^")
            ax.tick_params(axis="x", labelsize=20)
            ax.tick_params(axis="y", labelsize=20)
            ax.tick_params(axis="z", labelsize=20)
            plt.pause(1)
            ax.cla()
            # rate.sleep()
    except KeyboardInterrupt:
            plt.ioff()