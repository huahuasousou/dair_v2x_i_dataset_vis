import numpy as np
import re
from .ips_data_base import *
import os
import sys
import getopt

def get_arg():
    name = None
    url = None
    ipu_view=None
    camera_select=None #cam1 or cam2
    lidar_com=None #True select COM data

 
    argv = sys.argv[1:]
 
    try:
        opts, args = getopt.getopt(argv, "i:c:l:",  
                                   ["ipu=",
                                    "cam=",
                                    "lidar="])  # 长选项模式
     
    except:
        print("Error")
 
    for opt, arg in opts:
        if opt in ['-i', '--ipu']:
            ipu_view = arg
        elif opt in ['-c', '--cam']:
            camera_select = arg
        elif opt in ['-l', '--lidar']:
            lidar_com = arg     
 

    return ipu_view,camera_select,lidar_com


class IPS300DetectionDataset:
    def __init__(self,root_path,label_path = None,calib_path = None,ipu = None, cam = None, lidar_com = None):
        self.root_path = root_path
        self.calib_path = calib_path
        self.label_path = label_path
        
        self.label_name="_LABEL.txt"

        if lidar_com=="com":
            self.lidar_name="_COM_ROI.pcd"
            self.velo_path = os.path.join(self.root_path,"PCD_COM_ROI")
        if ipu=="1":
            self.cam_name="_IPU1"
            
            self.root_path=os.path.join(self.root_path,'IPU1')
            if lidar_com!="com":
                self.lidar_name="_IPU1_LIDAR.pcd"
                self.velo_path = os.path.join(self.root_path,"IPU1_pcd")
            if cam=="1":
                self.cam_name=self.cam_name+"_CAM1_UNDISTORT.jpg"
                self.image_path = os.path.join(self.root_path,"IPU1_cam1_undistort")
            if cam=="2":
                self.cam_name=self.cam_name+"_CAM2.png"
                self.image_path = os.path.join(self.root_path,"IPU1_cam2")  
        elif ipu=="2":
            self.root_path=os.path.join(self.root_path,'IPU2')
            if lidar_com!="com":
                               
                self.velo_path = os.path.join(self.root_path,"IPU2_pcd")
            if cam=="1":
                self.image_path = os.path.join(self.root_path,"IPU2_cam1")
            if cam=="2":
                self.image_path = os.path.join(self.root_path,"IPU2_cam2")    
        self.calib_name="calib_file.txt"
        self.all_ids = os.listdir(self.velo_path)
        if True:       
            print("root:",self.root_path) 
            print("label_path:",self.label_path)  
            print("velo_path:",self.velo_path)    
            print("image_path:",self.image_path)  
            print("calib_path:",self.calib_path)
             

    def __len__(self):
        #print("all_ids:",self.all_ids)
        return len(self.all_ids)
    def __getitem__(self, item):
        name = str(item).zfill(6)

        velo_path = os.path.join(self.velo_path,name+self.lidar_name)
        image_path = os.path.join(self.image_path, name+self.cam_name)
        calib_path = os.path.join(self.calib_path, self.calib_name)#未实现，修改
        label_path = os.path.join(self.label_path, name+self.label_name)

        P2,V2C = read_calib(calib_path)#仅实现ipu1cam1
        points = read_velodyne(velo_path,P2,V2C)#这里检查过没问题，对点云做了多次运算，为删除视野外点云。
        image = read_image(image_path)#仅支持去畸变后图像
        labels,label_names = read_detection_label(label_path)
        #labels[:,3:6] = cam_to_velo(labels[:,3:6],V2C)[:,:3]#这里已经给label做了一次变换了！也许kitti里label是在cam的？所以要转到lidar？注释掉就对了！然后角度还不太对。

        return P2,V2C,points,image,labels,label_names
