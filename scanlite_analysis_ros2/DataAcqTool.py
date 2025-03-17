#!/usr/bin/env python3

# ROS2 lib
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
# GUI and animation lib
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import os
import sys
# Image display lib
import matplotlib.pyplot as plt
import numpy as np
from tkinter import filedialog

# CATMAUS functions
paths_to_try = [
    os.path.dirname(os.path.abspath(__file__)),  # Current script directory
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "PyCATMAUS")
]

for path in paths_to_try:
    if os.path.exists(path) and path not in sys.path:
        sys.path.append(path)
        # Use ROS2 logging
        print(f"Added PyCATMAUS path: {path}")

try:
    from PyCATMAUS.SegBone import RunBoneSeg as BoneSeg
    from PyCATMAUS.TransFunction import quat2rotm, transfrom_i2l, imgp
    print("PyCATMAUS successfully loaded!")
except ModuleNotFoundError as e:
    print(f"PyCATMAUS import failed: {e}")

# File I/O lib
import pickle
import time
from copy import copy
from scipy.io import savemat

# Creates a ROS2 node with subscribers
class DataExchangeNode(Node):
    def __init__(self):
        super().__init__('cat_maus_app')
        self.img_subscriber = self.create_subscription(
            Image,
            '/us_image',
            self.update_img,
            10)
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/us_imu',
            self.update_imu,
            10)
        self.img = Image()
        self.imu = Imu()

    def update_img(self, data):
        self.img = data
        Data.img_buff = np.array(bytearray(self.img.data)).reshape(self.img.height, self.img.width, 4)
        # Data.add([timestamp, Data.img_buff, micronsPerPixel])

    def update_imu(self, data):
        self.imu = data
        # Data.addimu(to_add)

# class Data Storage is to be use to handle data with-in the app
class DataStore:
    # this class handles data and file functions
    def __init__(self):
        self.img_buff = np.zeros((480,640))
        self.data_buff = list()
        self.imu_buff = list()
    def reset(self):
        self.data_buff = list()
        self.imu_buff = list()
    def add(self,newlist):
        self.data_buff.append(newlist)
    def addimu(self,newimu):
        self.imu_buff.append(newimu)

class CatMausApp: 
    def __init__(self, master):
        self.master = master 
        ## GUI_Colours
        self._bg = '#d9d9d9'
        self.btn_colour_nor = '#cccccc'
        self.btn_colour_on  = '#229955'
        self.btn_colour_off = '#bbbbbb' 
        ## Create a container
        frame = tk.Frame(master=master, width=800, height=500,pady=5)
        master.resizable(0,1)
        master.geometry('1280x640+100+100')
        self.fig = plt.figure(figsize=(12.8,4.8), dpi=100, facecolor=self._bg, tight_layout=True)
        ## image axes
        ax1 = self.fig.add_subplot(121,title = "US Image", frameon = False)
        ax1.tick_params(axis='both',which='both',bottom=False,left=False,labelleft=False,labelbottom=False)
        ax2 = self.fig.add_subplot(122,projection='3d')
        init_img = np.zeros((480,640))
        self.USI1 = ax1.imshow(init_img,cmap='gray', vmin=0,vmax=255)
        self.USI2 = ax2.scatter([0],[0],[0],cmap='gray', vmin=0,vmax=255)
        self.Seg1, = ax1.plot([],[],'r.')
        self.canvas = FigureCanvasTkAgg(self.fig,master=master)
        self.canvas.draw()
    ## Functional variables (switcher index) 
        ## Switcher
        self.var_imgswitch = False
        self.var_con1 = False
        self.var_segswitch = False
        self.var_datarec = False
        self.var_replay_switch = False
        ## Variables
        self.save_1Var = tk.IntVar()
        self.save_2Var = tk.IntVar()
        self.var_Rind = tk.IntVar(value=0)
        self.var_DataCount = tk.IntVar(value=0)  
        self.var_file_name = tk.StringVar(value='US_data_0')
        ## Segmentation parameters
        self.var_f0 = tk.IntVar(value=100)
        self.var_f1 = tk.IntVar(value=50)
        self.var_bth= tk.IntVar(value=0.05)
        self.var_jc = tk.IntVar(value=2)
        ## Dynamic Buttons and labels
        self.button_connect1= tk.Button(frame,text="Connect",bg=self.btn_colour_off,command=self.Connect1,width=9)
        self.button_freeze= tk.Button(frame,text="Freeze",bg=self.btn_colour_off ,fg='white',command=self.Freeze,width=8)
        self.button_disconnect= tk.Button(frame,text="Disconnect",bg=self.btn_colour_nor,command=self.Disconnect,width=8)
        self.button_saveimgdata= tk.Button(frame,text="IMG Data",bg=self.btn_colour_nor ,command=self.saveimgdata,width=5)
        self.button_saveimudata= tk.Button(frame,text="IMU Data",bg=self.btn_colour_nor ,command=self.saveimudata,width=5)
        self.button_savepng= tk.Button(frame,text="PNG frame",bg=self.btn_colour_nor,command=self.savepng,width=5)
        self.button_segswitch= tk.Button(frame,text="BoneSeg",bg=self.btn_colour_off, fg='white',command=self.segswitchFn,width=5)
        # self.button_savedata= tk.Button(frame,text="PrintIMU",bg=self.btn_colour_off ,fg='white',command=self.printIMU,width=8)

        # recording button set
        self.button_recswitch= tk.Button(frame,text="Rec",bg=self.btn_colour_off,command=self.recswitchFn,width=5)
        self.button_replayswitch= tk.Button(frame,text="Play",bg=self.btn_colour_off,command=self.replay_switch_fn,width=5)
        
        ## Layout management
        self.canvas.get_tk_widget().grid(column=0,row=0,columnspan=12)
        #col0
        self.button_connect1.grid(row=0, column=0, columnspan=1, sticky='WE')
        self.button_freeze.grid(row=3,column=0,columnspan=1,sticky='WE')
        self.button_disconnect.grid(row=2,column=0,columnspan=1,sticky='WE')
        #col1
        tk.Label(frame,text = "  ").grid(row=1,column=1,sticky='E')
        #col2
        tk.Label(frame,text = "Viewing frame: ").grid(row=1,column=2,sticky='E')
        tk.Button(frame,text=" << ",command=self.R_backward,width=5).grid(row=2,column=2,sticky='E')
        tk.Label(frame,text="Save data as:").grid(row=3,column=2,sticky='E')
        tk.Label(frame,text="Data types:").grid(row=4,column=2,sticky='E')
        #col3
        tk.Label(frame,textvariable = self.var_Rind).grid(row=1,column=3,sticky='E')
        self.button_recswitch.grid(row=2,column=3,columnspan=1,sticky='WE')
        tk.Entry(frame,textvariable=self.var_file_name).grid(row=3,column=3,columnspan=3,sticky='we')
        self.button_saveimgdata.grid(row=4,column=3,columnspan=1,sticky='WE')        
        #col4
        tk.Label(frame,text=" of ").grid(row=1,column=4,sticky='W')
        tk.Label(frame,textvariable = self.var_DataCount).grid(row=1,column=4,sticky='E')
        self.button_replayswitch.grid(row=2,column=4,columnspan=1,sticky='WE')
        self.button_saveimudata.grid(row=4,column=4,columnspan=1,sticky='WE')
        #col5
        tk.Button(frame,text=" >> ",command=self.R_forward,width=5).grid(row=2,column=5,sticky='W')
        self.button_savepng.grid(row=4,column=5,columnspan=1,sticky='WE')
        #col6
        self.button_segswitch.grid(row = 1,column=8,sticky='w')
        tk.Label(frame,text='F0').grid(row=1,column=6,sticky='e')
        tk.Label(frame,text='F1').grid(row=2,column=6,sticky='e')        
        tk.Label(frame,text='BTh').grid(row=3,column=6,sticky='e')
        tk.Label(frame,text='JC').grid(row=4,column=6,sticky='e')
        tk.Entry(frame,width=4,textvariable=self.var_f0).grid(row=1,column=7,sticky='w')
        tk.Entry(frame,width=4,textvariable=self.var_f1).grid(row=2,column=7,sticky='w')
        tk.Entry(frame,width=4,textvariable=self.var_bth).grid(row=3,column=7,sticky='w')
        tk.Entry(frame,width=4,textvariable=self.var_jc).grid(row=4,column=7,sticky='w')

    ## Menu
        menu = tk.Menu(master)
        master.config(menu=menu)
        # File
        filemenu = tk.Menu(menu, tearoff=0)
        menu.add_cascade(label="File", menu=filemenu)
        # filemenu.add_command(label="New Collection", command=self.newsession)
        # filemenu.add_command(label="Save as image", command=self.saveone)
        # filemenu.add_command(label="Save as data file",command=self.saveall)
        #filemenu.add_command(label="Data Editor", command=0)
        filemenu.add_separator()
        filemenu.add_command(label="Exit", command=master.quit)
        # Edit
        editmenu = tk.Menu(menu, tearoff=0)
        menu.add_cascade(label="Edit", menu=editmenu)
        # editmenu.add_command(label="Segmentation editor", command=pop_editor)
        #editmenu.add_
        # Setting
        setmenu = tk.Menu(menu,tearoff = 0)
        menu.add_cascade(label="Settings",menu=setmenu)
        setmenu.add_command(label="Set US IP address")
        # Help
        helpmenu = tk.Menu(menu, tearoff=0)
        menu.add_cascade(label="Help", menu=helpmenu)
        # helpmenu.add_command(label="About...", command=lambda:msg_window(self.master,about_msg,'400x200'))

    ## Mount frame and mainloop
        frame.grid()
        self.master.mainloop()
    ##

    ## APP functions and callbacks
    def Connect1(self):
        if not self.var_con1:
            self.data_node = DataExchangeNode()
            self.var_con1 = True
            self.button_connect1.config(text="Connected", bg=self.btn_colour_on)
            self.var_imgswitch = True
            self.live_img()
        return

    def Disconnect(self):
        if self.var_con1:
            # reset interface
            self.var_con1 = False
            self.button_connect1.config(text="Connect", bg=self.btn_colour_off)
            self.var_imgswitch = False
            # disconnect data exchange node
            # to add codes
        return

    def Freeze(self):
        # to add functions
        if self.var_imgswitch:
            self.probe1.cast.userFunction(1, 0)
            self.var_imgswitch = False
            self.button_freeze.config(bg=self.btn_colour_on)
        else:
            self.probe1.cast.userFunction(1, 0)
            self.var_imgswitch = True
            self.button_freeze.config(bg=self.btn_colour_off)
            self.live_img()
        return

    def segswitchFn(self):
        if not self.var_segswitch:
            self.button_segswitch.config(text="BoneSeg", bg=self.btn_colour_on)
            self.var_segswitch = True
        else:
            self.button_segswitch.config(text="BoneSeg", bg=self.btn_colour_off)
            self.var_segswitch = False
            self.Seg1.set_xdata([])
            self.Seg1.set_ydata([])

    def recswitchFn(self):
        if not self.var_datarec:
            self.var_datarec = True
            self.button_recswitch.config(bg='red')            
            print("...recording started")
            Data.reset()
        else:
            self.var_datarec = False
            self.button_recswitch.config(bg=self.btn_colour_off)
            self.Freeze()
            self.data_out = copy(Data.data_buff)
            self.imu_out = copy(Data.imu_buff)
            self.var_DataCount.set(value=len(self.data_out))
            self.replay_img(0)
        return

    def live_img(self):
        if self.var_imgswitch:
            img_1 = Data.img_buff
            self.USI1.set_data(img_1)
            if self.var_segswitch is True:
                seg_tmp = BoneSeg(img_1[:, :, 0], self.var_f0.get(), self.var_f1.get(), self.var_bth.get(), self.var_jc.get())
                self.Seg1.set_xdata(seg_tmp[0])
                self.Seg1.set_ydata(seg_tmp[1])
            self.canvas.draw()
            self.master.after(30, self.live_img)
        else:
            return

    def replay_switch_fn(self):
        if not self.var_replay_switch:
            self.var_replay_switch = True
            self.button_replayswitch.config(bg=self.btn_colour_on)
            self.replay_loop()
        else:
            self.var_replay_switch = False
            self.button_replayswitch.config(bg=self.btn_colour_nor)

    def replay_loop(self):
        if self.var_replay_switch:
            self.R_forward()
            self.master.after(10, self.replay_loop)
        else:
            return

    def replay_img(self, n):
        self.var_Rind.set(value=n + 1)  # display frame number
        if len(self.data_out) != 0:
            img1 = self.data_out[n][1]
            self.USI1.set_data(img1)
        self.canvas.draw()

    def R_forward(self):
        n = self.var_Rind.get()
        if n < self.var_DataCount.get():
            self.replay_img(n)
        else:
            self.replay_img(0)

    def R_backward(self):
        n = self.var_Rind.get()
        if n - 2 >= 0:
            self.replay_img(n - 2)

    def saveimgdata(self):
        initfilename = self.var_file_name.get()
        Save_as_dat(self.data_out, initfilename)

    def saveimudata(self):
        initfilename = self.var_file_name.get()
        imu_array = np.array(self.imu_out)
        tm = imu_array[:, 0]
        acc = imu_array[:, 1:4]
        gyr = imu_array[:, 4:7]
        mag = imu_array[:, 7:10]
        quat = imu_array[:, 10:14]
        img_tm = imu_array[:, 14]
        file_name = tk.filedialog.asksaveasfilename(
            initialdir='/Users/catandmaus/Dropbox/OxfordPostdoc/CATMAUS/',
            initialfile=initfilename,
            filetypes=[('Data files', '*.mat')],
            defaultextension='.mat')
        if file_name:
            savemat(file_name, {
                "time": tm,
                "acc": acc,
                "gyr": gyr,
                "mag": mag,
                "quat": quat,
                "img_tm": img_tm
            })
        print("...Saved as: ", file_name)

    def savepng(self):
        initfilename = self.var_file_name.get()
        DispNum = self.var_Rind.get()
        svfile = initfilename + f"_{DispNum}"
        files = [('Portable image', '*.png')]
        filename = tk.filedialog.asksaveasfilename(filetypes=files, initialdir='data/', initialfile=svfile)
        img1 = self.USI1.get_array()
        plt.imsave(filename, img1, cmap='gray', dpi=100, format='png')

def main(args=None):
    rclpy.init(args=args)
    data_node = DataExchangeNode()
    rclpy.spin(data_node)
    data_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    # Data storage init
    Data = DataStore()
    root = tk.Tk()
    root.title("CAT&MAUS US DataAcq Single")
    APP = CatMausApp(root)
    main()
