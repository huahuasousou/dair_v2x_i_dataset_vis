import os
import cv2
import re
import open3d as o3d
import numpy as np
import json
"""
input: calib txt path
return: P2: (4,4) 3D camera coordinates to 2D image pixels
        vtc_mat: (4,4) 3D velodyne Lidar coordinates to 3D camera coordinates
"""
def read_calib(calib_path):

    K11=np.array((9.837163130824076e+02,0,0,\
        0,8.595545273466768e+02,0,\
        9.404020659139954e+02,5.756271947729268e+02,1),np.float32)


    K11=K11.reshape((3,3)).T
    K11= np.insert(K11, 3, values=0, axis=1)
    K11= np.insert(K11, 3, values=0, axis=0)    

    P11=np.array((0.14200562691952617, -0.9888265486076867, -0.045348193919961095, 1.0285320021923907,\
                -0.4095134968418579, -0.0169787561689188, -0.9121460506647551, -1.2267393975465035,\
                0.9011842751776943, 0.14810056923444437, -0.4073488966044802, -0.12756363884073307,     #-z    
                0, 0, 0, 1),np.float32)
    P11=P11.reshape((4,4))
    # IPU1_stereo.RotationOfCamera2
    IPU1CAM1TO2=np.array((0.998221611604617,-0.004574133046383,-0.059436448685599,-1.490257440631999e+02/100,
                0.001451114308713,0.998620862473591,-0.052481113743567,-3.327990406174097/100,
                0.059594533245466,0.052301532858762,0.996851564310293,13.818168849477330/100,
                0, 0, 0, 1),np.float32)
    IPU1CAM1TO2=IPU1CAM1TO2.reshape((4,4))
    #P11=np.dot(IPU1CAM1TO2,P11)#最像

    P21=np.array((0.14495024424955144, -0.9892874965026629, -0.017311858954031516, 0.24357166482617432,
                -0.4536471392935769, -0.05089834046335418, -0.8897267800757122, -0.6997266972502771,
                0.8793140848711282, 0.13681993153985403, -0.45616492144580906, 0.4865537741570236, 
                0.0, 0.0, 0.0, 1.0),np.float32)

      
    
   
    #P11[0:3,0:3]=P11[0:3,0:3].T

    return (K11, P11)


"""
description: read lidar data given 
input: lidar bin path "path", cam 3D to cam 2D image matrix (4,4), lidar 3D to cam 3D matrix (4,4)
output: valid points in lidar coordinates (PointsNum,4)
"""
def load_pcd_velo(velo_filename, n_vec=4):
    scan=o3d.io.read_point_cloud(velo_filename)
    scan=np.asarray(scan.points)
    scan=np.insert(scan, 3, values=0.2, axis=1)
    scan = scan.reshape((-1, n_vec))
    return scan

def read_velodyne(path, P, vtc_mat,IfReduce=True):
    max_row = 1080  # y
    max_col = 1920  # x
    lidar=load_pcd_velo(path)
    #lidar = np.fromfile(path, dtype=np.float32).reshape((-1, 4))
    if not IfReduce:
        return lidar

    mask = lidar[:, 0] > 0
    lidar = lidar[mask]
    
    lidar_copy = np.zeros(shape=lidar.shape)
    lidar_copy[:, :] = lidar[:, :]#读取拷贝一份点云list

    velo_tocam = vtc_mat#复制一份外参
    lidar[:, 3] = 1#点云xyz把强度值设为1，其实应该是为了接下来运算当作系数
    lidar = np.matmul(lidar, velo_tocam.T)#这里进行了一次运算！变换到3d相机坐标系了！跟住lidar看看去哪儿了
    img_pts = np.matmul(lidar, P.T)#变换到相机3d坐标系后，可以继续变换到2D图像坐标系了，应该是为了删除无用点云，跟住img_pts看看去哪儿了
    velo_tocam = np.mat(velo_tocam).I#这里还取逆是为了逆变换，注意T向量也取逆了，这个应该是争取的方式。R和转置效果一样，但是T向量只能取逆。之前的转制不是为了旋转R矩阵，只是为了批量运算
    velo_tocam = np.array(velo_tocam)#再转为数组
    normal = velo_tocam#这里又取了一次外参？？？
    normal = normal[0:3, 0:4]#删掉已经无用的最后一行0,0,0,1
    lidar = np.matmul(lidar, normal.T)#这里又做了一次逆运算,T是为了批量运算。又回到lidar坐标系了，验证此时数据恢复为最初，和copy一致
    lidar_copy[:, 0:3] = lidar#这里不明白，xyz重新赋值，最后一列强度值0.2也没有删掉，除了损失精度还有什么意义？
    x, y = img_pts[:, 0] / img_pts[:, 2], img_pts[:, 1] / img_pts[:, 2]#这个函数里面的2d运算看懂了，是为了取得视角mask，删掉无用点云，不涉及2d图像显示，但是会影响3d视野可见点云
    mask = np.logical_and(np.logical_and(x >= 0, x < max_col), np.logical_and(y >= 0, y < max_row))
    return lidar_copy#输出视野内不包括身后点云

    #return lidar_copy[mask]#暂时屏蔽，输出全部点云


"""
description: convert 3D camera coordinates to Lidar 3D coordinates.
input: (PointsNum,3)
output: (PointsNum,3)
"""
def cam_to_velo(cloud,vtc_mat):
    mat=np.ones(shape=(cloud.shape[0],4),dtype=np.float32)
    mat[:,0:3]=cloud[:,0:3]
    mat=np.mat(mat)
    normal=np.mat(vtc_mat).I
    normal=normal[0:3,0:4]
    transformed_mat = normal * mat.T
    T=np.array(transformed_mat.T,dtype=np.float32)
    return T

"""
description: convert 3D camera coordinates to Lidar 3D coordinates.
input: (PointsNum,3)
output: (PointsNum,3)
"""
def velo_to_cam(cloud,vtc_mat):
    mat=np.ones(shape=(cloud.shape[0],4),dtype=np.float32)
    mat[:,0:3]=cloud[:,0:3]
    mat=np.mat(mat)
    normal=np.mat(vtc_mat).I
    normal=normal[0:3,0:4]
    transformed_mat = normal * mat.T
    T=np.array(transformed_mat.T,dtype=np.float32)
    return T

def read_image(path):
    im=cv2.imdecode(np.fromfile(path, dtype=np.uint8), -1)
    return im



def read_json_detection_label(path):

    boxes = []
    names = []
    line = []

    with open(path) as load_f:
        load_dict = json.load(load_f)
        for line in load_dict['objects']:
            this_name=line['label']
            if this_name != "DontCare":
                dimensions=[(line['dimensions']['height'])]+[(line['dimensions']['width'])]+[(line['dimensions']['length'])]
                center=list((line['center'].values()))
                rotation_z=[(line['rotation']['z'])]
                line=dimensions+center+rotation_z
                line=np.array(line)
                line=line.astype(np.float32)
                #line= np.array((" ".join([str(i) for i in line]),np.float32))  #不需要转为空格分割了
                boxes.append(line)
                names.append(this_name)

    return np.array(boxes),np.array(names)



def read_detection_label(path):

    boxes = []
    names = []

    with open(path) as f:
        for line in f.readlines():
            line = line.split()
            this_name = line[0]
            if this_name != "DontCare":
                line = np.array(line[-7:],np.float32)
                boxes.append(line)
                names.append(this_name)

    return np.array(boxes),np.array(names)




def read_tracking_label(path):

    frame_dict={}

    names_dict={}

    with open(path) as f:
        for line in f.readlines():
            line = line.split()
            this_name = line[2]
            frame_id = int(line[0])
            ob_id = int(line[1])

            if this_name != "DontCare":
                line = np.array(line[10:17],np.float32).tolist()
                line.append(ob_id)


                if frame_id in frame_dict.keys():
                    frame_dict[frame_id].append(line)
                    names_dict[frame_id].append(this_name)
                else:
                    frame_dict[frame_id] = [line]
                    names_dict[frame_id] = [this_name]

    return frame_dict,names_dict

if __name__ == '__main__':
    path = 'H:/dataset/traking/training/label_02/0000.txt'
    labels,a = read_tracking_label(path)
    print(a)

