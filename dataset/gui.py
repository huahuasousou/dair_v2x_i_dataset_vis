from pyexpat.errors import XML_ERROR_XML_DECL
import numpy as np
import scipy.linalg as linalg
import math
import cv2


class setting_windows:
    def __init__(self,window_name):
        self.window_name=window_name
        self.pitch_degree=0
        self.yaw_degree=0

        cv2.namedWindow(self.window_name)
        #定义回调函数
        cv2.createTrackbar('X-roll (degree)',self.window_name,0,360,self.ax)
        #cv2.createTrackbar('Y-pitch (degree)',self.window_name,100,400,self.ay)
        #cv2.createTrackbar('Z-yaw (degree)',self.window_name,100,400,self.az)        

    def show_windows(self):
        while(1):
            #返回滑动条所在位置的值
            threshold1=cv2.getTrackbarPos('X-roll (degree)',self.window_name)
            #threshold2=cv2.getTrackbarPos('threshold2',self.window_name)
            #Canny边缘检测
            if cv2.waitKey(1)==ord('q'):
                break
    def ax(self,x):#在class中别忘了self传入
        self.roll_degree=x
        print("self.roll_degree: ",self.roll_degree)

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


