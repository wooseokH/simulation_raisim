##  /home/hanwooseok/raisim_ws/raisimLib/raisimUnity/linux/raisimUnity.x86_64
import os
import numpy as np
import raisimpy as raisim
import time
import threading
from control import control
from kinematics import kinematics
import plot
import matplotlib.pyplot as plt
from numpy.linalg import inv

def worldSetting():
    global world, server, ground, Ts, fs, vis_box,contact_box
    Ts=0.001
    fs=1/Ts
    
    world = raisim.World()
    server = raisim.RaisimServer(world)
    ground = world.addGround()
    world.setTimeStep(Ts)
    server.launchServer(8080)
    world.setERP(world.getTimeStep(), world.getTimeStep())
    # License
    license_path = "/home/ubuntu/raisim_ws/raisimLib/rsc/activation.raisim"
    raisim.World.setLicenseFile(license_path)

    # import wall
    # (x,y,z,colorR,colorG,colorB,colorA,material, glow, shadow)
    # vis_box=server.addVisualBox("v_box", 1.20, 1.50, 0.20, 1, 1, 1, 1,"steel",True)
    contact_box=world.addBox(0.2,1.5,1.5,100,"steel",1)
    # vis_box.setPosition(np.array([0.2,-0.35,0.0]))
    # visSphere.setPosition(np.array([2, 0, 0]))

def robotSetting():
    global mcl, non_linearities, mcl_dof

    # Manipulator 3DOF URDF
    mcl_urdf_file = "/home/ubuntu/raisim_ws/raisimLib/rsc/3DOF_before/urdf/mcl_3dof.urdf"

    # manipulator configuration
    mcl = world.addArticulatedSystem(mcl_urdf_file,"",[],1)
    mcl.setName("mcl")
    mcl_nominal_joint_config = np.array([0.0, 0.0, 0.0])
    mcl.setGeneralizedCoordinate(mcl_nominal_joint_config)
    mcl_dof=mcl.getDOF()
    
    ##### get dynamic properties #####
    # mass matrix


def variableInitialization():
    
    global run_time, run_time_idx, ext_force_time,t,ext_force_idx, safety_flag,run_time_arr,idx
    global des_pos, ee_pos,ext_force,s_tor
    global mass_matrix,inv_mass_matrix,nonlinear_matrix,l_ang2, l_vel2
    idx=0
    run_time=20.0
    run_time_idx=int(run_time*(fs))    # run time: [s] 
    ext_force_time=4.0
    run_time_arr=np.array([0.0]*run_time_idx,float)
    t=0.0
    ext_force_idx=0
    safety_flag=0
    # robot variable initialization
    global data_l_ang,data_m_tor
    data_l_ang=np.array([[0.0]*run_time_idx]*mcl_dof,dtype=np.float64)
    data_m_tor=np.array([[0.0]*run_time_idx]*mcl_dof,dtype=np.float64)

    mass_matrix=np.array([[0.0]*mcl_dof]*mcl_dof,dtype=np.float64)
    inv_mass_matrix=np.array([[0.0]*mcl_dof]*mcl_dof,dtype=np.float64)
    nonlinear_matrix=np.array([[0.0]*mcl_dof]*mcl_dof,dtype=np.float64)
    des_pos=np.array([0.0]*mcl_dof,dtype=float)
    ee_pos=np.array([0.0]*6,dtype=float)
    ext_force=np.array([0.0]*mcl_dof,dtype=float)

    
def waittingTime(waitting_time):
    print("\nWaitting..",waitting_time,"[s]\n\n")
    time.sleep(waitting_time)

def jointPositionControl():
    global cur_ang, desired_ang, u
    
    p_gain = np.array([70, 140, 110])
    d_gain = np.array([2, 2, 2])
    desired_ang=np.array([0.5, 0.4, 0.3])
    u, cur_ang=control.jointPID(mcl,desired_ang,p_gain,d_gain)
    nonlinear = mcl.getNonlinearities([0,0,-9.81])

    mcl.setGeneralizedForce(u+nonlinear)    # Joint input torque
    # control.safety_joint_ang(mcl)

def saveData():
    if mode == "joint_position_control":
        run_time_arr[i]=t
        data_l_ang[:,i]=cur_ang
        data_m_tor[:,i]=u
        
#     elif mode =="cartesian_impedance":
#         data_tor[:,i]=u
#         run_time[i]=t
#         data_pos[:,i]=ee_pos

#         data_ext_force[:,i]=ext_force

def plotGraph():
    if mode=="joint_position_control":
        fig_num=1
        plot.jointPos(fig_num,run_time_arr,data_l_ang,desired_ang)
        fig_num=2
        # plot.jointTor(fig_num,run_time_arr,data_tor)
    # elif mode=="cartesian_impedance":
    #     plot.eePos(run_time_arr,data_pos,desired_pos)
    #     plot.externalTorque(run_time_arr,data_ext_force)

    plt.show()


if __name__ == "__main__":
    # Initial setting
    worldSetting()

    robotSetting()

    variableInitialization()


    waittingTime(waitting_time=2)


    ############################## Control loop ##############################

    mode="joint_position_control"
    # mode="cartesian_impedance"
    cnt=0
    for i in range(run_time_idx):
        t1=time.time()
        t=Ts*i


        if mode == "joint_position_control":
            jointPositionControl()

        # elif mode == "cartesian_impedance":
        #     cartesianImpedanceControl()

       
        run_time_arr[i]=t

        world.integrate()
        t2=time.time()
        total_time=t2-t1
        saveData()
        # print("cnt,total time",cnt,total_time)
        if total_time<Ts:
            cnt+=1
            time.sleep(Ts-total_time)

    ###############################################################

    plotGraph()
    

    print("   SIMULATION END    ")


    server.killServer()


    # np.savetxt('./data/data_l_ang.txt',l_ang)
    # np.savetxt('./data/data_s_ang.txt',s_ang)
    
