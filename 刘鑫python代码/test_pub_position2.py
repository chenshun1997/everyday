from __future__ import print_function
import rospy
import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import time
import numpy as np
import math
import transformations as trans

import matplotlib.pyplot as plt
import pandas


def square(a):
    s = math.pow(a, 2)
    return s


class Queue:
    def __init__(self):
        self.items = []

    def isEmpty(self):
        return self.items == []

    def enqueue(self, item):
        self.items.insert(0, item)

    def dequeue(self):
        return self.items.pop()

    def size(self):
        return len(self.items)


def text_save(filename, data):  # filename为写入txt文件的路径，data为要写入数据列表.
    file = open(filename, 'w')
    for i in range(len(data)):
        s = str(data[i]).replace('[', '').replace(']', '')  # 去除[],这两行按数据不同，可以选择
        s = s.replace("'", '').replace(',', '') + '\n'  # 去除单引号，逗号，每行末尾追加换行符
        file.write(s)
    file.close()


if __name__ == '__main__':
    pos_pub = Queue()
    rospy.init_node('Pose_publisher2')

    while True:
        time_start = time.time()
        # print('time: ({})\n'.format(time.time()))
        for i in range(0, 10):
            pose = rospy.wait_for_message('/vrpn_client_node/crazyflie4/pose', geometry_msgs.PoseStamped, 1)
            pos_pub.enqueue(pose)
            if pos_pub.size() >= 10:
                pos_pub.dequeue()
            pub = rospy.Publisher('Pub_pos_cf2', nav_msgs.Path, queue_size=2)
            A_msg = Path()
            # 位置
            A_msg.poses = pos_pub.items
            pub.publish(A_msg)

            text_save('/home/l/cf2.txt', pos_pub.items)
        # time_end = time.time()            # 动捕系统测试
        # time_interval = time_end - time_start
        # max.append(time_interval)
        # time_max = np.array(max)
        #
        # if time_interval > 0.020:
        #     count += 1
        # times += 1
        # print('time_interval:', time_interval)
        # print('*************************************time_max:', np.max(time_max))
        # print('count:', count)
        # print('times', times)
