from viewer.viewer import Viewer
import numpy as np
from dataset.dair_dataset import DairDetectionDataset
import os
import yaml

def read_config(config_path='./config/config.yaml'):
    file = open(config_path, 'r', encoding="utf-8")
    #读取文件中的所有数据
    file_data = file.read()                 
    file.close()
    #指定Loader
    config_data = yaml.load(file_data,Loader=yaml.FullLoader)    
    return config_data


def dair_viewer(config_data):
           
    dataset = DairDetectionDataset(config_data)
    if dataset.label_select=='cam' or dataset.label_select=='vel' :
        vi = Viewer(box_type="Dair_V2X_I")
    elif dataset.label_select=='test':
        vi = Viewer(box_type="Dair_V2X_I_test")        
    vi.set_ob_color_map('gnuplot')
    #for i in range(len(dataset)):#这里修改文件名
    for i in dataset.all_ids:
        #i='005346'
        P2, V2C, points, image, labels, label_names = dataset[i]

        #mask = label_names=="Car"   #只显示Car
        #labels = labels[mask]           #labels是3dbounding box的7个值
        #label_names = label_names[mask] #是bounding box的类别
                                        #point是lidar的点

        vi.add_points(points[:,:3],scatter_filed=points[:,2],color_map_name='viridis')#原始lidar点，添加到2d和3d场景队列。不涉及任何转换
        vi.add_3D_boxes(labels,box_info=label_names,)#转换bounding box到vtk格式的线段和顶点list,问题出在读取label时候xyz就不对了，angle和whl都对，我知道了，按double读取int就导致xyz错误了！！！
        vi.add_3D_cars(labels, box_info=label_names)#只是为了添加3d车辆，没什么用
        vi.add_image(image,deep_copy=False)#只是添加图片，没什么看的

        vi.set_extrinsic_mat(V2C)#设置外参，也没什么看的
        vi.set_intrinsic_mat(P2)#设置内参，没什么问题
        
        vi.show_2D(label_select=dataset.label_select,index_name=i)
        vi.show_3D()


if __name__ == '__main__':

    config_data=read_config("./config/config.yaml")
    dair_viewer(config_data)

    