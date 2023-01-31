from scipy.spatial.transform import Rotation
import scipy.misc
import numpy as np
import time
import robotVar

class kinematics:
    def __init__(self):
        self.q0_null = np.array([0.0, 0.0, 0.0])

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
    def DH_Transform(cls,dh_par):
        alpha, a, d, theta=dh_par
        T = np.array([[np.cos(theta), -np.sin(theta), 0, a],
                      [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha),  -np.sin(alpha) * d],
                      [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha),   np.cos(alpha) * d],
                      [0, 0, 0, 1]])
        return T

    @classmethod
    def fk(cls, joints):
        dh_para=robotVar.DH_parameter(joints)
        T01=kinematics.DH_Transform(dh_para[0,:])
        T12=kinematics.DH_Transform(dh_para[1,:])
        T23=kinematics.DH_Transform(dh_para[2,:])
        T34=kinematics.DH_Transform(dh_para[3,:])
        
        Tbe=T01.dot(T12).dot(T23).dot(T34)
        return Tbe

    @classmethod
    def eePosition(cls, robot):       
        joint_frame_idx = robot.getFrameIdxByName('joint_ee')        
        ee_pos = robot.getFramePosition(joint_frame_idx)   
        ee_ori_mat = robot.getFrameOrientation(joint_frame_idx)      
        ee_ori = Rotation.from_matrix(ee_ori_mat).as_rotvec()
        x = np.concatenate((ee_pos, ee_ori))
        return x

    @classmethod
    def eeVelocity(cls, robot):
        JointFrameIndex = robot.getFrameIdxByName('joint_ee')  
        ee_vel = robot.getFrameVelocity(JointFrameIndex)  
        ee_ori_vel = robot.getFrameAngularVelocity(JointFrameIndex)
        x_dot = np.concatenate((ee_vel, ee_ori_vel)) 
        return x_dot

    @classmethod
    def getJacobian(cls, robot):
        Jv = robot.getDenseFrameJacobian("joint_ee")
        Jw = robot.getDenseFrameRotationalJacobian("joint_ee")
        J = np.vstack((Jv, Jw))
        return J

    # @classmethod
    # def jacobian(cls, joints):
    #     Tbe, Ts = pandaKinematics.fk(joints)
    #     Tbi = np.eye(4)
    #     J = np.zeros((6, 7))
    #     for i in range(7):
    #         Tbi = np.matmul(Tbi, Ts[i])
    #         Zi = Tbi[:3, 2]   # vector of actuation axis
    #         J[3:, i] = Zi   # Jw
    #         Pin = (Tbe[:3, -1] - Tbi[:3, -1])     # pos vector from (i) to (n)
    #         J[:3, i] = np.cross(Zi, Pin)  # Jv
    #     return J

    @classmethod
    def getJacobian2(cls, joints):
        dh_para=robotVar.DH_parameter(joints)
        T01=kinematics.DH_Transform(dh_para[0,:])
        T12=kinematics.DH_Transform(dh_para[1,:])
        T23=kinematics.DH_Transform(dh_para[2,:])
        T34=kinematics.DH_Transform(dh_para[3,:])
        
        Tbe=T01.dot(T12).dot(T23).dot(T34)
        T02=T01.dot(T12)
        T03=T01.dot(T12).dot(T23)


        P01=kinematics.fkMatrix2Vector(T01)[0:3]
        P02=kinematics.fkMatrix2Vector(T02)[0:3]
        P03=kinematics.fkMatrix2Vector(T03)[0:3]
        Pbe=kinematics.fkMatrix2Vector(Tbe)[0:3]
        print("P01",P01,P02,P03,Pbe)
        P0n=Pbe-P01
        P1n=Pbe-P02
        P2n=Pbe-P03
        print("P0n",P0n,P1n,P2n)

        Z0=T01[0:3,2]
        Z1=T02[0:3,2]
        Z2=T03[0:3,2]
        print("Z0",Z0,Z1,Z2)
        jacob=np.array([[0.0]*3]*3)
        jacob[:,0]=np.cross(Z0,P0n)
        jacob[:,1]=np.cross(Z1,P1n)
        jacob[:,2]=np.cross(Z2,P2n)
        print("jacob",np.cross(Z0,P0n),np.cross(Z1,P1n),np.cross(Z2,P2n))
        return jacob