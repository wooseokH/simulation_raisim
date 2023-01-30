import numpy as np
from numpy.linalg import inv
from kinematics import kinematics

class control:

    jacobian=np.zeros((3,3))
    prev_jacobian=np.zeros((3,3))

    x_dot=np.zeros((6))
    ee_acc=np.zeros((3))
    ee_vel=np.zeros((3))
    prev_ee_acc=np.zeros((3))
    prev_ee_vel=np.zeros((3))
    d_est=np.zeros((3,2))
    command_force=np.zeros((3))
    desired_force=np.zeros((3))    
    filtered_d_est=np.zeros((3,3))
    force_err=np.zeros((3,2))

    tau_u=1/(np.pi*2*0.5)
    tau_l=1/(np.pi*2*0.7)
    tau_q=1/(np.pi*2*3)

    def __init__(self):        
        pass
        
    ## Joint space PID control    
    @classmethod    
    def jointPD(cls, robot, des_ang,des_vel,p_gain,d_gain):       
        state = robot.getState()
        cur_ang=state[0][:robot.getDOF()]
        cur_vel=state[1][:robot.getDOF()]

        error = des_ang - cur_ang
        error_dot = des_vel- cur_vel
        command_torque = p_gain*error + d_gain*error_dot
        
        return command_torque, cur_ang

    
    ## Operating space control   
    @classmethod  
    def taskSpacePD(cls, robot, des_ee_pos,des_ee_vel,p_gain,d_gain):
        x = kinematics.eePosition(robot)
        x_dot = kinematics.eeVelocity(robot)
        ee_pos=x[:3]
        ee_vel=x_dot[:3]
        jacobian = robot.getDenseFrameJacobian("joint_ee_tip")
        
        err = des_ee_pos - ee_pos 
        err_dot = des_ee_vel - ee_vel       

        command_force = p_gain*err + d_gain*err_dot
        command_torque = (jacobian.T).dot(command_force.T) 

        return command_torque, ee_pos 


            
    @classmethod
    def forceControl(cls,robot,Ts,p_gain,d_gain,des_force,cur_force,jacobian):
        
        error=des_force+cur_force        
        command_force = p_gain*error
        command_torque=jacobian.T.dot(command_force)
    
        return command_torque,error

        
if __name__ == "__main__":    
    pass