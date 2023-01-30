from scipy.spatial.transform import Rotation
import scipy.misc
import numpy as np
import time


class kinematics:

    def __init__(self):        
        pass

    @classmethod
    def fkMatrix2Vector(cls, T):
        pos = T[:3, -1]
        rot = Rotation.from_matrix(T[:3, :3]).as_rotvec()     # for Python 3
        # rot = Rotation.from_dcm(T[:3, :3]).as_rotvec()  # for Python2
        return np.concatenate((pos, rot))


    @classmethod
    def fkVector2Matrix(cls, x):
        T = np.zeros((4,4))
        T[:3, -1] = x[:3]
        T[:3, :3] = Rotation.from_rotvec(x[3:]).as_matrix()
        return T


    @classmethod
    def dhModifiedTransform(cls,dh_par):
        alpha, a, d, theta=dh_par
        T = np.array([[np.cos(theta), -np.sin(theta), 0, a],
                      [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha),  -np.sin(alpha) * d],
                      [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha),   np.cos(alpha) * d],
                      [0, 0, 0, 1]])
        return T


    @classmethod
    def dhStandardTransform(cls,dh_par):
        alpha, a, d, theta=dh_par
        T = np.array([[np.cos(theta),   -np.cos(alpha)*np.sin(theta),   np.sin(alpha)*np.sin(theta),    a*np.cos(theta)],
                      [np.sin(theta),   np.cos(alpha) * np.cos(theta),  -np.sin(alpha)*np.cos(theta),   a*np.sin(theta)],
                      [0,               np.sin(alpha),                  np.cos(alpha),                  d              ],
                      [0,               0,                              0,                              1              ]])
        return T



    @classmethod
    def fk(cls, joints):
        L0=0.136
        L1=0.149
        L2=0.45
        L3=0.249
        L4=0.105
        q1=joints[0]
        q2=joints[1]
        q3=joints[2]
        dh_par1=[0,        0,    L0,     q1]
        dh_par21=[0,        0,    L1,     0]
        dh_par2=[-np.pi/2, 0,    0,          q2-90*np.pi/180]
        dh_par3=[0 ,       L2,   0,        q3]
        dh_par4=[0 ,       L3,   0,        0]
        dh_par5=[0 ,       L4,   0,        0]
        
        T01=kinematics.dhModifiedTransform(dh_par1)
        T11=kinematics.dhModifiedTransform(dh_par21)
        T12=kinematics.dhModifiedTransform(dh_par2)
        T23=kinematics.dhModifiedTransform(dh_par3)
        T34=kinematics.dhModifiedTransform(dh_par4)   
        T45=kinematics.dhModifiedTransform(dh_par5)   

        Tbe=T01.dot(T11).dot(T12).dot(T23).dot(T34).dot(T45)
        return Tbe


    @classmethod
    def eePosition(cls, robot):       
        joint_frame_idx = robot.getFrameIdxByName('joint_ee_tip')        
        # joint_frame_idx = robot.getFrameIdxByName('joint_ee')        
        
        ee_pos = robot.getFramePosition(joint_frame_idx)   
        ee_ori_mat = robot.getFrameOrientation(joint_frame_idx)      
        ee_ori = Rotation.from_matrix(ee_ori_mat).as_rotvec()
        x = np.concatenate((ee_pos, ee_ori))
        return x

    @classmethod
    def eeVelocity(cls, robot):
        JointFrameIndex = robot.getFrameIdxByName('joint_ee_tip')  
        # JointFrameIndex = robot.getFrameIdxByName('joint_ee')  
        ee_vel = robot.getFrameVelocity(JointFrameIndex)  
        ee_ori_vel = robot.getFrameAngularVelocity(JointFrameIndex)
        x_dot = np.concatenate((ee_vel, ee_ori_vel)) 
        return x_dot

    @classmethod
    def getJacobian(cls, robot):
        Jv = robot.getDenseFrameJacobian("joint_ee_tip")
        Jw = robot.getDenseFrameRotationalJacobian("joint_ee_tip")
        # Jv = robot.getDenseFrameJacobian("joint_ee")
        # Jw = robot.getDenseFrameRotationalJacobian("joint_ee")


        J = np.vstack((Jv, Jw))
        return J


