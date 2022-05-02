import numpy as np
import re
from .dair_data_base import *
import os
import sys
import getopt


def traverse(f):# list dir and sort
    fs = os.listdir(f)
    fs.sort()
    return fs

def namechange(name_list,new_suffix):# change the suffix
    new_name=[]
    for name in name_list:
        portion = os.path.splitext(name)
        new_name.append(portion[0] + new_suffix)
    
    return new_name

class DairDetectionDataset:
    def __init__(self,config_data= None):
        self.root_path = os.path.join(config_data['root_path'],config_data['output_floder_name'])
        self.label_select= config_data['label_select']
        #self.label_path='/home/liuchenghua/Downloads/0428/data'
        
        if self.label_select=='cam':
            self.label_path=os.path.join(self.root_path,"label_0410")
        elif self.label_select=='vel':
            self.label_path=os.path.join(self.root_path,"label_velodyne")             
        
        
   
        self.calib_path=os.path.join(self.root_path,"calib_originP23X3")
        self.velo_path = os.path.join(self.root_path,"velodyne")     
        self.image_path = os.path.join(self.root_path,"image_2")   
        self.label_name=".txt"
        self.lidar_name=".pcd"
        self.cam_name=".jpg"
        self.calib_name=".txt"

        self.all_ids = traverse(self.label_path)
        self.all_ids=namechange(self.all_ids,"")#delete the suffix
        print("type",type(self.all_ids))

        #self.all_ids = os.path.splitext(self.all_ids)
        #self.all_ids=self.all_ids[0]
        #print(self.all_ids)

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
        velo_path = os.path.join(self.velo_path, name+self.lidar_name)
        image_path = os.path.join(self.image_path, name+self.cam_name)
        calib_path = os.path.join(self.calib_path, name+self.calib_name)
        label_path = os.path.join(self.label_path, name+self.label_name)


        P2,V2C = read_calib(calib_path)#
        points = read_velodyne(velo_path,P2,V2C)#这里检查过没问题，对点云做了多次运算，为删除视野外点云。
        image = read_image(image_path)#仅支持去畸变后图像
        labels,label_names = read_detection_label(label_path)
        #labels[:,3:6] = cam_to_velo(labels[:,3:6],V2C)[:,:3]#这里已经给label做了一次变换了！也许kitti里label是在cam的？所以要转到lidar？注释掉就对了！然后角度还不太对。

        return P2,V2C,points,image,labels,label_names
