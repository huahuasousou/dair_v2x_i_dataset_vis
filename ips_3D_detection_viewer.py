from viewer.viewer import Viewer
import numpy as np
from dataset.ips_dataset import IPS300DetectionDataset
from dataset.ips_dataset import get_arg
import os

def ips_viewer(ipu,cam,lidar_com):
    root="/home/liuchenghua/IPS300plus_object_vis/data/IPS300+/IPS300+_detection/data"
    label_path = "/home/liuchenghua/IPS300plus_object_vis/data/IPS300+/IPS300+_detection/label/txt"
    calib_path="/home/liuchenghua/IPS300plus_object_vis/data/IPS300+/IPS300+_detection/calib"
                        
    dataset = IPS300DetectionDataset(root,label_path,calib_path,ipu,cam,lidar_com)

    vi = Viewer(box_type="Kitti")
    vi.set_ob_color_map('gnuplot')
    print("len(dataset):",dataset[1])
    for i in range(len(dataset)):
        P2, V2C, points, image, labels, label_names = dataset[i]

        mask = label_names=="Minibus"  #只显示Car
        labels = labels[mask]
        label_names = label_names[mask]#感觉显示lidar有问题，lidar没转换或者转换错了，否则不可能box和lidar对不上

        vi.add_points(points[:,:3],scatter_filed=points[:,2],color_map_name='viridis')
        vi.add_3D_boxes(labels,box_info=label_names)
        vi.add_3D_cars(labels, box_info=label_names)
        vi.add_image(image)
        vi.set_extrinsic_mat(V2C)
        vi.set_intrinsic_mat(P2)
        vi.show_2D()
        vi.show_3D()


if __name__ == '__main__':


    ipu_view=None
    camera_select=None #cam1 or cam2
    lidar_com=None #True select COM data    
    ipu_view,camera_select,lidar_com=get_arg()
    ips_viewer(ipu_view,camera_select,lidar_com)

    