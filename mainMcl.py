##  /home/hanwooseok/raisim_ws/raisimLib/raisimUnity/linux/raisimUnity.x86_64
import os
import numpy as np
import raisimpy as raisim
import time
import threading
from control import control
from kinematics import kinematics
from myPlot import myPlot
import matplotlib.pyplot as plt
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R

def worldSetting():
    global world, server, ground, Ts, fs,sim_time

    Ts=0.001
    fs=1/Ts
    sim_time=10.0
    world = raisim.World()
    ground = world.addGround()
    world.setTimeStep(Ts)
    world.setERP(world.getTimeStep(), world.getTimeStep())

    server = raisim.RaisimServer(world)
    server.launchServer(8080)

    # License
    license_path = "/home/ubuntu/.raisim/activation.raisim"
    raisim.World.setLicenseFile(license_path)

    # add wall for contact simulation
    wall=world.addBox(0.1, 1.60, 1.0, 100)
    wall.setPosition(0.55, -0.0, 0.6/2.0)  # 

    # trajectroy line
    global refLine
    refLine=server.addVisualPolyLine("refLine")
    refLine.setColor(1,0,0,1)
    

def robotSetting():
    global mcl, non_linearities, mcl_dof

    # URDF
    mcl_urdf_file = "/home/ubuntu/raisim_ws/raisimLib/rsc/MCL_3DOF_2/MCL_3DOF/urdf/lab_3dof (plus_ball).urdf"
       
    
    # manipulator configuration
    mcl = world.addArticulatedSystem(mcl_urdf_file)
    mcl.setName("mcl")
    # mcl_nominal_joint_config = np.array([0.0,0,0])
    mcl_nominal_joint_config = np.array([0.0, 0.22, 1.4])
    mcl.setGeneralizedCoordinate(mcl_nominal_joint_config)
    mcl_dof=mcl.getDOF()
    

    # Print body and frame names
    print("Body name in order:")
    mcl.printOutBodyNamesInOrder()
    print("\nFrame name in order:")
    mcl.printOutFrameNamesInOrder() 

def variableInitialization():
    

    ##################### system variable #####################
    global sim_time_idx,t,idx,RAD_TO_DEG,DEG_TO_RAD
    sim_time_idx=int(sim_time*(fs))    
    t=0.0
    idx=0
    DEG_TO_RAD=np.pi/180
    RAD_TO_DEG=180/np.pi
    
    ##################### robot variable #####################
    global mass_matrix,inv_mass_matrix,nonlinear_matrix,nonlinear_joint
    mass_matrix=np.zeros((mcl_dof,mcl_dof),dtype=np.float64)
    inv_mass_matrix=np.zeros((mcl_dof,mcl_dof),dtype=np.float64)
    nonlinear_matrix=np.zeros((mcl_dof,mcl_dof),dtype=np.float64)

    global jacobian,prev_jacobian,jacobian_dot
    jacobian=np.zeros((3,mcl_dof),dtype=np.float64)
    prev_jacobian=np.zeros((3,mcl_dof),dtype=np.float64)
    jacobian_dot=np.zeros((3,mcl_dof),dtype=np.float64)
    nonlinear_joint=np.zeros((mcl_dof),dtype=np.float64)

    ##################### control variable #####################
    global des_ang, des_pos, des_vel, des_acc, ee_pos, command_torque
    des_ang=np.array([-0*DEG_TO_RAD, 40*DEG_TO_RAD, 30*DEG_TO_RAD],dtype=np.float64)
    des_pos=np.zeros((mcl_dof),dtype=np.float64)
    des_vel=np.zeros((mcl_dof),dtype=np.float64)
    des_acc=np.zeros((mcl_dof),dtype=np.float64)
    ee_pos=np.zeros((mcl_dof,2),dtype=np.float64)
    command_torque=np.zeros((mcl_dof),dtype=np.float64)
    
    global p_gain,d_gain
    p_gain = np.array([0.0, 0.0, 0.0])
    d_gain = np.array([0.0, 0.0, 0.0])  

    # DOB
    global est_d
    est_d=np.zeros((3),dtype=np.float64)


    ##################### contact variable #####################
    global contact_frame,contact_force,filtered_contact_force
    contact_frame=np.zeros((3,3),dtype=np.float64)
    contact_force=np.zeros((3,2),dtype=np.float64)
    filtered_contact_force=np.zeros((3,2),dtype=np.float64)

    ##################### trajectroy #####################
    global a0, a1, a2, a3, init_pos,init_vel,final_pos,final_vel,traj_pos,traj_vel,traj_acc
    a0=np.zeros((3),dtype=np.float64) 
    a1=np.zeros((3),dtype=np.float64)  
    a2=np.zeros((3),dtype=np.float64)  
    a3=np.zeros((3),dtype=np.float64)
    init_pos=np.zeros((3),dtype=np.float64) 
    final_pos=np.zeros((3),dtype=np.float64) 
    init_vel=np.zeros((3),dtype=np.float64)
    final_vel=np.zeros((3),dtype=np.float64) 
    traj_pos=np.zeros((3,sim_time_idx),dtype=np.float64)
    traj_vel=np.zeros((3,sim_time_idx),dtype=np.float64)
    traj_acc=np.zeros((3,sim_time_idx),dtype=np.float64)


    ##################### data variable #####################
    # system
    global data_time 
    data_time=np.zeros((sim_time_idx),dtype=np.float64)
    
    # robot
    global data_nonlinear_joint
    data_nonlinear_joint=np.zeros((mcl_dof,sim_time_idx),dtype=np.float64)

    # control
    global data_l_ang,data_m_tor,data_desired_force,data_err_force,data_des_pos,data_ee_pos,data_command_torque
    data_l_ang=np.zeros((mcl_dof,sim_time_idx),dtype=np.float64)
    data_m_tor=np.zeros((mcl_dof,sim_time_idx),dtype=np.float64)
    data_desired_force=np.zeros((mcl_dof,sim_time_idx),dtype=np.float64)
    data_err_force=np.zeros((mcl_dof,sim_time_idx),dtype=np.float64)
    data_des_pos=np.zeros((mcl_dof,sim_time_idx),dtype=np.float64)
    data_ee_pos=np.zeros((mcl_dof,sim_time_idx),dtype=np.float64)
    data_command_torque=np.zeros((mcl_dof,sim_time_idx),dtype=np.float64)
    
    global desired_force
    desired_force=np.array([10.0, 0.0, 0.0],dtype=np.float64)

    # contact
    global data_contact_force
    data_contact_force=np.zeros((mcl_dof,sim_time_idx),dtype=np.float64)

    # DOB
    global data_est_distub,data_control_force
    data_est_distub=np.zeros((mcl_dof,sim_time_idx),dtype=np.float64)
    data_control_force=np.zeros((mcl_dof,sim_time_idx),dtype=np.float64)


def trajectory():
    global traj_pos,traj_vel,traj_acc
        
    # initial condition
    init_pos[:]=[0.4,0.0,0.7]
    final_pos[:]=[0.6,0.0,0.7]
    init_vel[:]=[0.0,0.0,0.0]
    final_vel[:]=[0.0,0.0,0.0]
    final_time=sim_time/2
    traj_time = np.arange(start=0.0, stop=final_time, step=Ts)

    # qubic trajectroy
    a0=init_pos
    a1=init_vel
    a2 = 3*(final_pos-init_pos)/(final_time**2) - 2/final_time*init_vel - 1/final_time*final_vel
    a3 = -2*(final_pos-init_pos)/(final_time**3) + 1/(final_time**2)*(init_vel+final_vel)
    
    traj_pos[0,:len(traj_time)] = a0[0] + a1[0]*traj_time[:] + a2[0]*traj_time[:]**2 + a3[0]*traj_time[:]**3
    traj_pos[1,:len(traj_time)] = a0[1] + a1[1]*traj_time + a2[1]*traj_time**2 + a3[1]*traj_time**3
    traj_pos[2,:len(traj_time)] = a0[2] + a1[2]*traj_time + a2[2]*traj_time**2 + a3[2]*traj_time**3
    traj_vel[0,:len(traj_time)] = a1[0] + 2*a2[0]*traj_time + 3*a3[0]*traj_time**2
    traj_vel[1,:len(traj_time)] = a1[1] + 2*a2[1]*traj_time + 3*a3[1]*traj_time**2
    traj_vel[2,:len(traj_time)] = a1[2] + 2*a2[2]*traj_time + 3*a3[2]*traj_time**2
    traj_acc[0,:len(traj_time)] = 2*a2[0] + 6*a3[0]*traj_time
    traj_acc[1,:len(traj_time)] = 2*a2[1] + 6*a3[1]*traj_time
    traj_acc[2,:len(traj_time)] = 2*a2[2] + 6*a3[2]*traj_time

    # maintain final position
    traj_pos[0,len(traj_time):len(traj_pos[0,])]=final_pos[0]    
    traj_pos[1,len(traj_time):len(traj_pos[0,])]=final_pos[1]    
    traj_pos[2,len(traj_time):len(traj_pos[0,])]=final_pos[2]    

    # trajectroy line
    for i in range(sim_time_idx):
        refLine.addPoint(traj_pos[:,i])



def waittingTime(waitting_time):
    print("\nWaitting..",waitting_time,"[s]\n\n")
    time.sleep(waitting_time)

def dataUpdate():
    ee_pos[:,1]=ee_pos[:,0]
    contact_force[:,1]=contact_force[:,0]
    filtered_contact_force[:,1]=filtered_contact_force[:,0]

def jointPositionControl():
    global cur_ang, des_ang, command_torque, nonlinear_joint
    
    p_gain = np.array([150, 150, 150])
    d_gain = np.array([10, 10, 10])
    des_ang=[-0*DEG_TO_RAD, 0*DEG_TO_RAD, 0*DEG_TO_RAD]

    # control joint PD
    command_torque, cur_ang=control.jointPD(mcl,des_ang,des_vel,p_gain,d_gain)
    nonlinear_joint = mcl.getNonlinearities()

    # input torque
    mcl.setGeneralizedForce(command_torque+nonlinear_joint)    

def cartesianImpedanceControl():
    global ee_pos, des_pos,prev_jacobian,est_d,nonlinear_joint,command_torque
    
    
    p_gain[:] = [50, 450, 450]
    d_gain[:] = [20, 20, 20]
    jacobian = mcl.getDenseFrameJacobian("joint_ee_tip")
    jacobian_dot=control.derivativeOfMat(Ts,prev_jacobian,jacobian)
    prev_jacobian=jacobian

    # task space PD control
    command_torque,ee_pos[:,0],=control.taskSpacePD(mcl,traj_pos[:,i],traj_vel[0,i],p_gain,d_gain)

    # prevent Nan value
    if np.isnan(command_torque).any():
        command_torque[:]=[0,0,0]

    # nonlinear term
    nonlinear_joint = mcl.getNonlinearities([0,0,-9.81])
    
    # set external force
    # mcl.setExternalForce(3,[0,0,0.3],[0,1*np.sin(np.pi*2*0.1*t),0])

    mcl.setGeneralizedForce(command_torque+nonlinear_joint)     
  
def forceControl():
    global ee_pos, des_pos,prev_jacobian,est_d,nonlinear_joint,command_torque,error
    # desired cartesian position
    desired_force[:]=[10.0, 0.0, 0.0]
    p_gain[:] = [20, 20, 20]
    d_gain[:] = [0, 0, 0]
    jacobian = mcl.getDenseFrameJacobian("joint_ee_tip")
    
    command_torque[:],error=control.forceControl(mcl,Ts,p_gain,d_gain,desired_force,filtered_contact_force[:,0],jacobian)
    
    x = kinematics.eePosition(mcl)
    ee_pos[:,0]=x[:3]

    nonlinear_joint = mcl.getNonlinearities([0,0,-9.81])

    mcl.setGeneralizedForce(command_torque+nonlinear_joint)


def isContact():
    global filtered_contact_force
    contacts = mcl.getContacts()

    if len(contacts)>1:     # if contacted
        contact_frame=contacts[0].getContactFrame()
        contact_force[:,0]=contact_frame.T.dot(contacts[0].getImpulse()/Ts)

        # LPF filter
        tau=1/(2*np.pi*5)
        filtered_contact_force[:,0]=(Ts*(contact_force[:,0]+contact_force[:,1])+(-Ts+2*tau)*filtered_contact_force[:,1])/(2*tau+Ts)

    else :
        contact_force[:,0]=[0.0,0.0,0.0]
        filtered_contact_force[:,0]=[0.0,0.0,0.0]        


def saveData():
    global contact_force,data_nonlinear_joint,command_torque,filtered_contact_force
    if mode == "joint_position_control":
        data_time[i]=t
        data_l_ang[:,i]=cur_ang
        data_m_tor[:,i]=command_torque
            
    elif mode =="cartesian_impedance":
        data_time[i]=t
        data_contact_force[:,i]=-filtered_contact_force[:,0]
        data_desired_force[:,i]=desired_force[:]
        data_est_distub[:,i]=est_d
        data_des_pos[:,i]=traj_pos[:,i]
        data_ee_pos[:,i]=ee_pos[:,0]
        data_command_torque[:,i]=command_torque
        data_nonlinear_joint[:,i]=nonlinear_joint

    elif mode =="force_control":
        data_time[i]=t
        data_contact_force[:,i]=filtered_contact_force[:,0]
        data_desired_force[:,i]=desired_force[:]
        data_ee_pos[:,i]=ee_pos[:,0]
        data_err_force[:,i]=error


############## CONTROL LOOP END ##############


def saveToFile():
    with open('./data/data_time.txt','w') as f1:
        np.savetxt(f1,data_time.T)
    with open('./data/data_contact_force.txt','w') as f2:
        np.savetxt(f2,data_contact_force.T)
    with open('./data/data_des_pos.txt','w') as f3:
        np.savetxt(f3,data_des_pos.T)
    with open('./data/data_ee_pos.txt','w') as f4:
        np.savetxt(f4,data_ee_pos.T)

    f1.close(); f2.close(),f3.close(); f4.close()



def plotGraph():
    myPlot.plot3(1,data_time,data_contact_force,data_contact_force,'Contact Force of End Effector','time[s]',["x[N]","y[N]","z[N]"],["contact_force","desired_force"])
    myPlot.plot3(2,data_time,data_ee_pos,data_des_pos,'Trajectory of End Effector','time [s]',["x[m]","y[m]","z[m]"],["current","desired"])   
    myPlot.plot3(3,data_time,data_est_distub,data_est_distub,'d hat','time [s]',["","",""],["",""])   
    plt.show()


if __name__ == "__main__":
    # Initial setting
    worldSetting()

    robotSetting()

    variableInitialization()

    trajectory()

    waittingTime(waitting_time=2)


    ################################### CONTROL LOOP START #########################################

    mode="force_control"
    mode="joint_position_control"
    mode="cartesian_impedance"
    cnt=0

    for i in range(sim_time_idx):
        t1=time.time()
        t=Ts*i

        dataUpdate()

        isContact()

        if mode == "joint_position_control":
            jointPositionControl()

        elif mode == "cartesian_impedance":
            cartesianImpedanceControl()
        
        elif mode =="force_control":
            forceControl()

        saveData()
        t2=time.time()
        calculation_time=t2-t1

        world.integrate()

        # satisfied in saimpling time 
        if calculation_time<Ts:
            cnt+=1
            time.sleep(Ts-calculation_time)
        if calculation_time>Ts:
            print("Calculation time(",calculation_time,") over Ts(",Ts,")")

        
    ################################### CONTROL LOOP END #########################################

    saveToFile()
    
    plotGraph()

    print("   SIMULATION END    ")

    server.killServer()