import os, sys
import shutil
import scipy.io as scio 
import sys
import yaml
import shutil
import numpy as np
import pandas as pd
from random import shuffle
import random

def read_config(config_path='./config/config.yaml'):
    file = open(config_path, 'r', encoding="utf-8")
    #读取文件中的所有数据
    file_data = file.read()                 
    file.close()
    #指定Loader
    config_data = yaml.load(file_data,Loader=yaml.FullLoader)    
    return config_data

class convert_datasets:
    def __init__(self,config_data):
        np.set_printoptions(linewidth=1000)#防止numpy转换str自动换行
        self.config_data=config_data
        self.output_path = os.path.join(config_data['output_path'],config_data['output_floder_name'])

        #self.data_info_path=os.path.join(config_data['root_path'],config_data['data_info'])
        #self.data_info=read_config(data_info_path)
        self.source_image_path=os.path.join(config_data['root_path'],config_data['image_floder'])
        self.source_velodyne_path=os.path.join(config_data['root_path'],config_data['velodyne_floder'])
        self.source_calib_path=os.path.join(config_data['root_path'],config_data['calib_floder'])
        self.source_camera_Tr_path=os.path.join(config_data['root_path'],config_data['lidar_to_camera_floder'])
        self.source_camera_label=os.path.join(config_data['root_path'],config_data['camera_label_floder'])
        self.source_velodyne_label=os.path.join(config_data['root_path'],config_data['virtuallidar_label_floder'])

        self.calib_path=os.path.join(self.output_path,"calib")
        self.image2_path=os.path.join(self.output_path,"image_2")
        self.label2_path=os.path.join(self.output_path,"label_2")
        self.label_vel_path=os.path.join(self.output_path,"label_velodyne")
        self.velodyne_path=os.path.join(self.output_path,"velodyne")
        self.splitting_path=os.path.join(self.output_path,"splitting")
        
        self.train_set_proportion=config_data['train_set_proportion']
        self.validation_set_proportion=config_data['validation_set_proportion']
        self.test_set_proportion=config_data['test_set_proportion']
        self.random_seed=config_data['random_seed']
        self.onlyFileName=config_data['onlyFileName']
        
        self.create_floders_list=[]
        self.create_floders_list.append(self.calib_path)
        #self.create_floders_list.append(self.image2_path)
        self.create_floders_list.append(self.label2_path)
        self.create_floders_list.append(self.label_vel_path)
        self.create_floders_list.append(self.splitting_path)        
        
        #self.create_floders_list.append(self.velodyne_path)
        #print("create_floders list:",self.create_floders_list)
        
        #self.files_list = os.listdir(self.source_image_path)
        #self.files_list.sort()
        #self.files_len=len(self.files_list)
        self.car_count=0
        self.van_count=0

    #创建目录，返回路径
    def create_path(self):
        for floder in self.create_floders_list:        
            if not os.path.exists(floder):
                os.makedirs(floder)
                print("created the path:",floder)

    def check_files(self):
        #check image, velodyne and label are correspond.
        pass

    def calib_convert(self):
        output_path=self.calib_path
        calib_input_path=self.source_calib_path
        tr_input_path=self.source_camera_Tr_path
        
        self.calib_list = os.listdir(self.source_calib_path)
        self.calib_list.sort()          #no neceressory, save time
        for index in self.calib_list:
            calib=read_config(os.path.join(calib_input_path,index))
            tr=read_config(os.path.join(tr_input_path,index))

            rotation_matrix=np.mat(tr["rotation"])
            translation_matrix=np.mat(tr["translation"])
            Tr_matrix=np.c_[rotation_matrix,translation_matrix]          
            Tr_matrix=Tr_matrix.flatten()
            Tr_matrix=str(Tr_matrix)
            Tr_matrix=Tr_matrix.strip("[]")#delete "[" and "]"
            Tr_matrix=Tr_matrix.replace("  "," ")# replace the "," to " "
            #matrix="P2: "+matrix
            Tr_matrix="Tr_velo_to_cam: "+Tr_matrix
            #print(index,Tr_matrix)
            
            #P2_matrix=str(calib["P"])#not P matrix
            P2_matrix=str(calib["cam_K"])# should use cam_K matrix
            P2_matrix=P2_matrix.strip("[]")
            P2_matrix=P2_matrix.replace(",","")# replace the "," to " "
            P2_matrix="P2: "+P2_matrix

            output_name = os.path.splitext(index)
            with open(os.path.join(output_path,output_name[0]+".txt"), 'wt') as f:
                print("write file:",os.path.join(output_path,output_name[0]+".txt"))
                f.write("P0: \n")
                f.write("P1: \n")
                f.write(P2_matrix+"\n")
                f.write("P3: \n")
                f.write("R0_rect: \n")
                f.write(Tr_matrix)

             
                            
    def label_convert(self,input_path,output_path):
        delete_class=['Trafficcone','Tricyclists','Barrowlists']#Set the list you want to delete
        merge_class={'Car':['Van']}#Set the list you want to merge,example 'a':[1, 2, 3],'b':[13, 22, 32],'c':[15, 'sdf']

        files_list = os.listdir(input_path)
        files_list.sort()
        #print(files_list)
        
        for index in files_list:
            label=read_config(os.path.join(input_path,index))
            output_name = os.path.splitext(index)
            with open(os.path.join(output_path,output_name[0]+".txt"), 'wt') as f:
                print("write file:",os.path.join(output_path,output_name[0]+".txt"))
                for line in label:
                    #print("line: ",line,"\n")
                    f.write(line['type']+" ")
                    f.write(line['truncated_state']+" ")
                    f.write(line['occluded_state']+" ")
                    f.write(line['alpha']+" ")
                    f.write(line['2d_box']['xmin']+" ")
                    f.write(line['2d_box']['ymin']+" ")
                    f.write(line['2d_box']['xmax']+" ")
                    f.write(line['2d_box']['ymax']+" ")

                    f.write(line['3d_dimensions']['h']+" ")
                    f.write(line['3d_dimensions']['w']+" ")
                    f.write(line['3d_dimensions']['l']+" ")

                    f.write(line['3d_location']['x']+" ")
                    f.write(line['3d_location']['y']+" ")
                    f.write(line['3d_location']['z']+" ")

                    f.write(line['rotation']+" "+"\n")
    def label_convert_fix_high(self,camera_label_path,lidar_label_path,output_path):
        delete_class=['Trafficcone','Tricyclists','Barrowlists']#Set the list you want to delete
        merge_class={'Car':['Van']}#Set the list you want to merge,example 'a':[1, 2, 3],'b':[13, 22, 32],'c':[15, 'sdf']

        '''
        for key,value in merge_class.items():
            print(key,value)    
        '''

        lidar_files_list = os.listdir(lidar_label_path)
        lidar_files_list.sort()
        camera_files_list = os.listdir(camera_label_path)
        camera_files_list.sort()        
        #print(files_list)
        self.car_count=0
        self.van_count=0
        
        for index in lidar_files_list:
            lidar_label=read_config(os.path.join(lidar_label_path,index))
            camera_label=read_config(os.path.join(camera_label_path,index))
                        
            output_name = os.path.splitext(index)
            with open(os.path.join(output_path,output_name[0]+".txt"), 'wt') as f:
                print("write file:",os.path.join(output_path,output_name[0]+".txt"))
                write_buffer=""
                for camera_line,lidar_line in zip(camera_label,lidar_label):

                    if camera_line['type'] in delete_class:
                        #print("Skip the class",camera_line['type'])
                        pass
                    else:
                        for key,value in merge_class.items(): 
                            if camera_line['type'] in value:
                                #print("replace the class",camera_line['type'], "to",key)
                                camera_line['type']=key

                        

                        if camera_line['type']=="Car":
                            self.car_count=self.car_count+1
                        elif camera_line['type']=="Van":
                            self.van_count=self.van_count+1
                        write_buffer+=camera_line['type']+" "
                        write_buffer+=camera_line['truncated_state']+" "
                        write_buffer+=camera_line['occluded_state']+" "
                        write_buffer+=camera_line['alpha']+" "
                        write_buffer+=camera_line['2d_box']['xmin']+" "
                        write_buffer+=camera_line['2d_box']['ymin']+" "
                        write_buffer+=camera_line['2d_box']['xmax']+" "
                        write_buffer+=camera_line['2d_box']['ymax']+" "

                        write_buffer+=lidar_line['3d_dimensions']['h']+" "
                        write_buffer+=lidar_line['3d_dimensions']['w']+" "
                        write_buffer+=lidar_line['3d_dimensions']['l']+" "

                        write_buffer+=camera_line['3d_location']['x']+" "
                        write_buffer+=camera_line['3d_location']['y']+" "
                        write_buffer+=lidar_line['3d_location']['z']+" "

                        write_buffer+=camera_line['rotation']+" "+"\n"
                        #print("write_buffer:",write_buffer)
                f.write(write_buffer)
        print("car_count",self.car_count)
        print("van_count",self.van_count)
                    
    def rename_floder(self,input_path,output_path):
        print("copy file from ",input_path,"to the ",output_path)
        shutil.copytree(input_path,output_path)
        
    def creat_list(self):
        
        random_seed=self.random_seed
        onlyFileName=self.onlyFileName
        random.seed(int(random_seed))
        r=random.random()
        
        output_path=self.splitting_path
        print("datasets splitting output path: ",output_path)
        camera_files_list = os.listdir(self.image2_path)
        #random.shuffle(camera_files_list,random=r)#打乱list
        shuffle(camera_files_list)#打乱list
        #camera_files_list.sort()        
        #print("camera_list: ",camera_files_list)
        datasets_size=len(camera_files_list)
        print("datasets_size: ",len(camera_files_list))
        
        train_sets=camera_files_list[0:int(self.train_set_proportion*datasets_size)]
        val_sets=camera_files_list[int(self.train_set_proportion*datasets_size):int(self.train_set_proportion*datasets_size)+int(self.validation_set_proportion*datasets_size)]
        test_sets=camera_files_list[int(self.train_set_proportion*datasets_size)+int(self.validation_set_proportion*datasets_size):int(self.train_set_proportion*datasets_size)+int(self.validation_set_proportion*datasets_size)+int(self.test_set_proportion*datasets_size)+1]
        """
        print("train_sets:",train_sets)
        print("val_sets:",val_sets)
        print("test_sets:",test_sets)        
        """
        train_output_path=os.path.join(output_path,"train.txt")
        
        val_output_path=os.path.join(output_path,"val.txt")
        
        test_output_path=os.path.join(output_path,"test.txt")
                
        with open(train_output_path, 'wt') as f:
            print("write file:",train_output_path)
            for index in train_sets:
                if onlyFileName== True:
                    image_name=index
                    lidar_name=os.path.splitext(index)[0]+".pcd"
                    cam_label_name=os.path.splitext(index)[0]+".txt"
                    vel_label_name=os.path.splitext(index)[0]+".txt"
                    write_col=lidar_name+" "+vel_label_name+" "+image_name+" "+cam_label_name+"\n"
              
                else:
                    image_name=os.path.join(self.image2_path,index)
                    lidar_name=os.path.join(self.velodyne_path,os.path.splitext(index)[0]+".pcd")
                    cam_label_name=os.path.join(self.label2_path,os.path.splitext(index)[0]+".txt")
                    vel_label_name=os.path.join(self.label_vel_path,os.path.splitext(index)[0]+".txt")                    
                    write_col=lidar_name+" "+vel_label_name+" "+image_name+" "+cam_label_name+"\n"                   
                #print(write_col)
                f.write(write_col)  

        with open(val_output_path, 'wt') as f:
            print("write file:",val_output_path)
            for index in val_sets:
                if onlyFileName== True:
                    image_name=index
                    lidar_name=os.path.splitext(index)[0]+".pcd"
                    cam_label_name=os.path.splitext(index)[0]+".txt"
                    vel_label_name=os.path.splitext(index)[0]+".txt"
                    write_col=lidar_name+" "+vel_label_name+" "+image_name+" "+cam_label_name+"\n"
              
                else:
                    image_name=os.path.join(self.image2_path,index)
                    lidar_name=os.path.join(self.velodyne_path,os.path.splitext(index)[0]+".pcd")
                    cam_label_name=os.path.join(self.label2_path,os.path.splitext(index)[0]+".txt")
                    vel_label_name=os.path.join(self.label_vel_path,os.path.splitext(index)[0]+".txt")                    
                    write_col=lidar_name+" "+vel_label_name+" "+image_name+" "+cam_label_name+"\n"                   
                #print(write_col)
                f.write(write_col)  

        with open(test_output_path, 'wt') as f:
            print("write file:",test_output_path)
            for index in test_sets:
                if onlyFileName== True:
                    image_name=index
                    lidar_name=os.path.splitext(index)[0]+".pcd"
                    cam_label_name=os.path.splitext(index)[0]+".txt"
                    vel_label_name=os.path.splitext(index)[0]+".txt"
                    write_col=lidar_name+" "+vel_label_name+" "+image_name+" "+cam_label_name+"\n"
              
                else:
                    image_name=os.path.join(self.image2_path,index)
                    lidar_name=os.path.join(self.velodyne_path,os.path.splitext(index)[0]+".pcd")
                    cam_label_name=os.path.join(self.label2_path,os.path.splitext(index)[0]+".txt")
                    vel_label_name=os.path.join(self.label_vel_path,os.path.splitext(index)[0]+".txt")                    
                    write_col=lidar_name+" "+vel_label_name+" "+image_name+" "+cam_label_name+"\n"                   
                #print(write_col)
                f.write(write_col) 
                
                
                
                
    def creat_image_list(self):
        
        random_seed=self.random_seed
        onlyFileName=self.onlyFileName
        random.seed(int(random_seed))
        r=random.random()
        
        output_path=self.splitting_path
        print("datasets splitting output path: ",output_path)
        camera_files_list = os.listdir(self.image2_path)
        #random.shuffle(camera_files_list,random=r)#打乱list
        shuffle(camera_files_list)#打乱list
        #camera_files_list.sort()        
        #print("camera_list: ",camera_files_list)
        datasets_size=len(camera_files_list)
        print("datasets_size: ",len(camera_files_list))
        
        train_sets=camera_files_list[0:int(self.train_set_proportion*datasets_size)]
        val_sets=camera_files_list[int(self.train_set_proportion*datasets_size):int(self.train_set_proportion*datasets_size)+int(self.validation_set_proportion*datasets_size)]
        test_sets=camera_files_list[int(self.train_set_proportion*datasets_size)+int(self.validation_set_proportion*datasets_size):int(self.train_set_proportion*datasets_size)+int(self.validation_set_proportion*datasets_size)+int(self.test_set_proportion*datasets_size)+1]
        """
        print("train_sets:",train_sets)
        print("val_sets:",val_sets)
        print("test_sets:",test_sets)        
        """
        train_output_path=os.path.join(output_path,"train.txt")
        
        val_output_path=os.path.join(output_path,"val.txt")
        
        test_output_path=os.path.join(output_path,"test.txt")
                
        with open(train_output_path, 'wt') as f:
            print("write file:",train_output_path)
            for index in train_sets:
                image_name=os.path.join(self.config_data['image_list_define_path'],index)
                cam_label_name=os.path.join(self.config_data['image_label_list_path'],os.path.splitext(index)[0]+".txt")                    
                write_col=image_name+" "+cam_label_name+"\n"                   
                print(write_col)
                f.write(write_col)  

        with open(val_output_path, 'wt') as f:
            print("write file:",val_output_path)
            for index in val_sets:
                image_name=os.path.join(self.config_data['image_list_define_path'],index)
                cam_label_name=os.path.join(self.config_data['image_label_list_path'],os.path.splitext(index)[0]+".txt")                    
                write_col=image_name+" "+cam_label_name+"\n"                   
                print(write_col)
                f.write(write_col)   

        with open(test_output_path, 'wt') as f:
            print("write file:",test_output_path)
            for index in test_sets:
                image_name=os.path.join(self.config_data['image_list_define_path'],index)
                cam_label_name=os.path.join(self.config_data['image_label_list_path'],os.path.splitext(index)[0]+".txt")                    
                write_col=image_name+" "+cam_label_name+"\n"                   
                print(write_col)
                f.write(write_col)                 



if __name__ == '__main__':


    #You can select which folders you need to convert by commenting out parts of the code

    config_data=read_config('./config/config.yaml')    #read config file get the convert input and output path
    dair2kitti=convert_datasets(config_data)    #instantiate the object
    dair2kitti.create_path()    #create the subfloders
    dair2kitti.calib_convert()    #calib file convert to the kitti format

    #camera coordinate label convert, but origin camera label have some issues, so don't use this.
    #dair2kitti.label_convert(dair2kitti.source_camera_label,dair2kitti.label2_path)   

    #camera coordinate label convert, and fix the camera label issues
    #base the camera label source, and replace the pos(z) and the size(l,w,h) by the lidar label source.
    dair2kitti.label_convert_fix_high(dair2kitti.source_camera_label,dair2kitti.source_velodyne_label,dair2kitti.label2_path)

    #velodyne coordinate label convert
    dair2kitti.label_convert(dair2kitti.source_velodyne_label,dair2kitti.label_vel_path)


    #Make sure the destination folder does not exist before converting

    #copy and rename the camera image floder
    dair2kitti.rename_floder(dair2kitti.source_image_path,dair2kitti.image2_path)

    #copy and rename the velodyne pcd floder
    dair2kitti.rename_floder(dair2kitti.source_velodyne_path,dair2kitti.velodyne_path)
    #convert finished        