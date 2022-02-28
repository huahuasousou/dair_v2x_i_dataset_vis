from viewer.viewer import Viewer
import numpy as np
from dataset.dair_dataset import DairDetectionDataset
from dataset.dair_dataset import get_arg
import os

def dair_viewer(ipu,cam,lidar_com):
    root="/home/liuchenghua/IPS300plus_object_vis/data/IPS300+/IPS300+_detection/data"
    label_path = "/home/liuchenghua/IPS300plus_object_vis/data/IPS300+/IPS300+_detection/label1/txt"
    calib_path="/home/liuchenghua/IPS300plus_object_vis/data/IPS300+/IPS300+_detection/calib"
                        
    dataset = DairDetectionDataset(root,label_path,calib_path,ipu,cam,lidar_com)

    vi = Viewer(box_type="Dair_V2X_I")
    vi.set_ob_color_map('gnuplot')
    for i in range(len(dataset)):
        P2, V2C, points, image, labels, label_names = dataset[i]
        #P2, V2C, points, image, labels, label_names = dataset[1]

        mask = label_names=="Minibus"   #只显示Car
        labels = labels[mask]           #labels是3dbounding box的7个值
        label_names = label_names[mask] #是bounding box的类别
                                        #point是lidar的点

        vi.add_points(points[:,:3],scatter_filed=points[:,2],color_map_name='viridis')#原始lidar点，添加到2d和3d场景队列。不涉及任何转换
        vi.add_3D_boxes(labels,box_info=label_names)#转换bounding box到vtk格式的线段和顶点list,问题出在读取label时候xyz就不对了，angle和whl都对，我知道了，按double读取int就导致xyz错误了！！！
        #vi.add_3D_cars(labels, box_info=label_names)#只是为了添加3d车辆，没什么用
        vi.add_image(image,deep_copy=False)#只是添加图片，没什么看的
        #从这里截断，循环设置外参
  
        #V2C[0,3]+=i
        vi.set_extrinsic_mat(V2C)#设置外参，也没什么看的
        vi.set_intrinsic_mat(P2)#设置内参，没什么问题
        
        vi.show_2D()


        #print(i)
        #i+=1
        vi.show_3D()


if __name__ == '__main__':


    label_select=None   #select camera or velodyne coordinate
   
    label_select=get_arg()
    dair_viewer(label_select)

    