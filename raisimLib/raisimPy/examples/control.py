import numpy as np
from kinematics import kinematics
class control:
   
    def __init__(selKineticsf):        
        pass
        
    ## Joint space PID control    
    @classmethod    
    def jointPID(cls, robot, desired_pos,p_gain,d_gain):       
        state = robot.getState()
        cur_ang=state[0][:robot.getDOF()]

        error = desired_pos - cur_ang
        error_dot = - state[1][:robot.getDOF()]
        pid_torque = p_gain*error + d_gain*error_dot
        
        u = pid_torque
        return u, cur_ang
       
    
    
    ## Operating space control   
    @classmethod  
    def taskSpacePD(cls, robot, ee_desired,p_gain,d_gain):          
        x = kinematics.eePosition(robot)
        xd = kinematics.eeVelocity(robot)
        ee_pos=x[:3]
        ee_vel=xd[:3]
        J = robot.getDenseFrameJacobian("joint_ee")

        error = ee_desired - ee_pos 
        error_dot = - ee_vel       
        F = p_gain*error + d_gain*error_dot

        Tau = (J.T).dot(F.T) 
        u = Tau
        
        return u, ee_pos

    @classmethod  
    def safety_joint_ang(cls,robot): 
        state = robot.getState()
        cur_ang=state[0][:robot.getDOF()]
        safety_flag=0
        if cur_ang[1]>1.8:
            safety_flag=1
            print("joint 1 angle limit")
            cur_ang[1]=1.8
        elif cur_ang[1]<-1.8:
            safety_flag=1
            print("joint 1 angle limit")
            cur_ang[1]=-1.8

        if cur_ang[2]>1.6:
            safety_flag=1
            print("joint 2 angle limit")
            cur_ang[2]=1.6
        elif cur_ang[2]<-1.6:
            safety_flag=1
            print("joint 2 angle limit")
            cur_ang[2]=-1.6
        # if cur_ang[2]    
        # if cur
        return safety_flag
        
    @classmethod
    def extForceLim(cls,ext_force):
        ext_force_lim=np.array([40,40,40])
        if(ext_force[0]>ext_force_lim[0]):
            ext_force[0]=ext_force_lim[0]; 
        if(ext_force[1]>ext_force_lim[0]):
            ext_force[1]=ext_force_lim[2]; 
        if(ext_force[2]>ext_force_lim[2]):
            ext_force[2]=ext_force_lim[2];             

        return ext_force
        
if __name__ == "__main__":    
    pass