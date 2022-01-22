from viewer.viewer import Viewer
import numpy as np
from dataset.ips_dataset import IPS300DetectionDataset
from dataset.ips_dataset import get_arg
import os
from dataset.gui import *

def ips_viewer(ipu,cam,lidar_com):
    root="/home/liuchenghua/IPS300plus_object_vis/data/IPS300+/IPS300+_detection/data"
    label_path = "/home/liuchenghua/IPS300plus_object_vis/data/IPS300+/IPS300+_detection/label1/txt"
    calib_path="/home/liuchenghua/IPS300plus_object_vis/data/IPS300+/IPS300+_detection/calib"
                        
    dataset = IPS300DetectionDataset(root,label_path,calib_path,ipu,cam,lidar_com)

    vi = Viewer(box_type="IPS300")
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

        angle_matrix=rotationMatrixToEulerAngles(V2C[0:3,0:3])
        print(angle_matrix)
        print(V2C[0:3,3])

        print(eulerAnglesToRotationMatrix([math.radians(160.02016116),math.radians(-64.31417419),math.radians(-70.87516021)]))
        print(V2C[0:3,0:3])
        vi.set_intrinsic_mat(P2)#设置内参，没什么问题

        if True:    #设置面板
            setting_matrix=setting_windows("setting_windows")
        
        setting_matrix.show_windows()
        #从这里截断，循环设置外参
        while(True):
            vi.add_image(image,deep_copy=True)#只是添加图片，没什么看的
            vi.set_extrinsic_mat(V2C)#设置外参，也没什么看的

            #vi.show_2D()
            #vi.show_3D()


if __name__ == '__main__':


    ipu_view=None
    camera_select=None #cam1 or cam2
    lidar_com=None #True select COM data    
    ipu_view,camera_select,lidar_com=get_arg()
    ips_viewer(ipu_view,camera_select,lidar_com)

    