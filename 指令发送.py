#!/usr/bin/env python
import tkinter as tk
import numpy as np
from tkinter import *
from tkinter import messagebox
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
# 创建画布需要的库
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# import rospy
master = tk.Tk()
master.title("无人机控制指令的输入")
master.geometry('1200x350')

# frame1 = Frame(master, bg="#ffffff")
# frame1.place(x=5, y=320, width=1200, height=700)

# plt.rcParams['font.sans-serif'] = ['SimHei'] 
# fig = plt.figure(figsize=(10, 7), edgecolor='blue')
# ax = Axes3D(fig)
# ax.set_xlim(0, 2000)
# ax.set_ylim(1000, 0)
# ax.set_zlim(0, 1000)
# ax.set_xlabel("x")
# ax.set_ylabel("y")
# ax.set_zlabel("z")
# canvas = FigureCanvasTkAgg(fig, master=frame1)
# canvas.draw()
# canvas.get_tk_widget().place(x=0, y=0)



#标签
tk.Label(master, text="请输入无人机的类别（A、B、C、D、E）：").grid(row=0)
tk.Label(master, text="请分别输入集群个数（1～8）：").grid(row=1)
tk.Label(master, text="所对应的集群队形（a一字型、b三角形、c正方形）：").grid(row=2)
tk.Label(master, text="其他：").grid(row=3)
 
e0 = tk.Entry(master, show=None)
e1 = tk.Entry(master, show=None)
e2 = tk.Entry(master, show=None)
e3 = tk.Entry(master, show=None)

e0.grid(row=0, column=1, padx=10, pady=5)
e1.grid(row=1, column=1, padx=10, pady=5)
e2.grid(row=2, column=1, padx=10, pady=5)
e3.grid(row=3, column=1, padx=10, pady=5)


 #目的是进行打印，并对输入框进行清空
def show():
    
    print('无人机的类别\t\t集群个数\t\t集群队形')
    lia=f'{e0.get()}'
    lib=f'{e1.get()}'
    lic=f'{e2.get()}'
    lid=f'{e3.get()}'
    li0 = lia.split()
    li1 = lib.split()
    li2 = lic.split()
    li3 = lid.split()

    # for a in li0:
    #     print(a,end='\t\t\t\t')
    #     for b in li1:
    #         print(b,end='\t\t')
    #         for c in li2:
    #             print(c,end='\t\t')
    print(f'{li0[0]}\t\t\t\t{li1[0]}\t\t\t\t{li2[0]}')
    print(f'{li0[1]}\t\t\t\t{li1[1]}\t\t\t\t{li2[1]}')
    print(f'{li0[2]}\t\t\t\t{li1[2]}\t\t\t\t{li2[2]}')
    # print(li1[1])
    # print(li2[1])

    # print(li0[1])
    # print(lis)
    # s = vars()['e0.get()']
    # the = s.split()


    print("无人机的类别：%s" % e0.get())
    print("每种的集群个数：%s" % e1.get())
    print("所对应的集群队形：%s" % e2.get())
    print("其他：%s" % e3.get())

    e0.delete(0, "end")
    e1.delete(0, "end")
    e2.delete(0, "end")
    e3.delete(0, "end")


t = tk.Text(master,height=5,width=50)

# t.tag_config("tag_1", background="yellow", foreground="red")
t.grid(row=10,column=1)
# t.insert('insert', "无人机的类别：\n","tag_1")
# t.insert('insert', "\t无人机的类别\t\t集群个数\t\t集群队形\n")
t.insert('insert', "无人机的类别:\n")
t.insert('insert', "每种的集群数：\n")
t.insert('insert', "对应集群队形：\n")
t.insert('insert', "其他：\n")



def insert_point1():
	t.mark_set("here","1.9")
	t.insert('here', e0.get())
	t.mark_unset("here")
	#t.mark_set("e0","1.2")
    #t.insert('end', )
def insert_point2():
    t.mark_set("here","2.8")
    t.insert('here', e1.get())
    t.mark_unset("here")
#每次选完数据后的确认 .grid(row=10)
def insert_point3():
    t.mark_set("here","3.9")
    t.insert('here', e2.get())
    t.mark_unset("here")

def insert_point4():
    t.mark_set("here","4.9")
    t.insert('here', e3.get())
    t.mark_unset("here")

tk.Button(master, text='确定', width=10,command=insert_point1).grid(row=0, column=2, padx=10, pady=5)
tk.Button(master, text='确定', width=10,command=insert_point2).grid(row=1, column=2, padx=10, pady=5)
tk.Button(master, text='确定', width=10,command=insert_point3).grid(row=2, column=2, padx=10, pady=5)
tk.Button(master, text='确定', width=10,command=insert_point4).grid(row=3, column=2, padx=10, pady=5)

#最下方的确认 
tk.Button(master, text="发送给控制台", width=10, command=show).grid(row=5, column=0, sticky="w", padx=10, pady=5)
tk.Button(master, text="退出", width=10, command=master.quit).grid(row=5, column=1, sticky="e", padx=10, pady=5)
# if __name__ == '__main__':
    # main()
# point = [[0,0,0],[1, 2, 3], [2, 3, 4], [3, 4, 5]]
# point1 = [[1, 2, 3], [2, 3, 4], [3, 4, 5],[0,0,0]]
# # while True:
    
#     for i in range(len(point)):
#         # init()
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
master.mainloop()
