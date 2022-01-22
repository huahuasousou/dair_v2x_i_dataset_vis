from pyexpat.errors import XML_ERROR_XML_DECL
import numpy as np
import scipy.linalg as linalg
import math
import cv2


class setting_windows:
    def __init__(self,window_name):
        self.window_name=window_name
        self.roll_degree=160.02016116  
        self.pitch_degree=360-64.31417419
        self.yaw_degree=360-70.87516021
        self.vect_x=500+1.028532 *1000/20
        self.vect_y=500-1.2267394*1000/20
        self.vect_z=500+0.12756364*1000/20

        self.T=np.array((0,0,0),np.float32) 
        self.T=self.T.reshape((3,1))

        self.R=np.array((0,0,0,0,0,0,0,0,0),np.float32) 
        self.R=self.R.reshape((3,3))        
        self.P=np.array((0.14200562691952617, -0.9888265486076867, -0.045348193919961095, 1.0285320021923907,\
                    -0.4095134968418579, -0.0169787561689188, -0.9121460506647551, -1.2267393975465035,\
                    0.9011842751776943, 0.14810056923444437, -0.4073488966044802, -0.12756363884073307,     #-z    
                    0, 0, 0, 1),np.float32)
        self.P=self.P.reshape((4,4))

        cv2.namedWindow(self.window_name)

        #设定各滑块
        cv2.createTrackbar('X-roll (degree)',self.window_name,0,360,self.ax)
        cv2.createTrackbar('Y-pitch (degree)',self.window_name,0,360,self.ay)
        cv2.createTrackbar('Z-yaw (degree)',self.window_name,0,360,self.az)
        cv2.createTrackbar('X-vect (m)',self.window_name,0,1000,self.vx)
        cv2.createTrackbar('Y-vect (m)',self.window_name,0,1000,self.vy)
        cv2.createTrackbar('Z-vect (m)',self.window_name,0,1000,self.vz)
        #设定初始值，init函数可以读取文件到self中，这样可以自动加载
        cv2.setTrackbarPos('X-roll (degree)',self.window_name, int(self.roll_degree))
        cv2.setTrackbarPos('Y-pitch (degree)',self.window_name,int(self.pitch_degree))
        cv2.setTrackbarPos('Z-yaw (degree)',self.window_name,int(self.yaw_degree))
        cv2.setTrackbarPos('X-vect (m)',self.window_name,int(self.vect_x))
        cv2.setTrackbarPos('Y-vect (m)',self.window_name,int(self.vect_y))
        cv2.setTrackbarPos('Z-vect (m)',self.window_name,int(self.vect_z))  

    def update_matrix(self):
        self.R=eulerAnglesToRotationMatrix([math.radians(self.roll_degree),math.radians(self.pitch_degree),math.radians(self.yaw_degree)])
        self.T=(20*(self.vect_x-500)/1000,20*(self.vect_y-500)/1000,20*(self.vect_z-500)/1000)#归一化为+-10米
        self.P[0:3,0:3]=self.R
        self.P[0:3,3]=self.T
        print(self.P)
    def show_windows(self):
        #while(1):
        #返回滑动条所在位置的值
        threshold1=cv2.getTrackbarPos('X-roll (degree)',self.window_name)
        #threshold2=cv2.getTrackbarPos('threshold2',self.window_name)
        #Canny边缘检测
        #if cv2.waitKey(1)==ord('q'):
            #break
    def get_matrix(self):
        return self.P


    def ax(self,input_value):#在class中别忘了self传入
        self.roll_degree=input_value
        self.update_matrix()

    def ay(self,input_value):#在class中别忘了self传入
        self.pitch_degree=input_value
        self.update_matrix()

    def az(self,input_value):#在class中别忘了self传入
        self.yaw_degree=input_value
        self.update_matrix()
    def vx(self,input_value):#在class中别忘了self传入
        self.vect_x=input_value
        self.update_matrix()


    def vy(self,input_value):#在class中别忘了self传入
        self.vect_y=input_value
        self.update_matrix()


    def vz(self,input_value):#在class中别忘了self传入
        self.vect_z=input_value
        self.update_matrix()

    def read_config(self):
        pass
    def save_config(self):
        pass
         

def eulerAnglesToRotationMatrix(theta) :
    
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
        
        
                    
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                    
                    
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

def rotationMatrixToEulerAngles(R) :

    #assert(isRotationMatrix(R))
    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([math.degrees(x), math.degrees(y), math.degrees(z)])
    #return np.array([x, y, z])


