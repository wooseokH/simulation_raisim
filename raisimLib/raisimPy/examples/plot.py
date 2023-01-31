import numpy as np
import matplotlib.pyplot as plt

def jointPos(fig_num,data_time,data_pos,target_pos):
    plt.figure(fig_num)
    plt.subplot(311)
    plt.plot(data_time,data_pos[0,:])
    plt.plot(data_time,np.array([target_pos[0]]*np.size(data_time)),'r--')
    plt.xlabel('time[s]')
    plt.ylabel('angle[rad]')
    # plt.legend(('joint 1','ref'))
    plt.grid(True)
    #plt.ylim(-0.2,0.7)
    plt.title('Joint Position')
    plt.subplot(312)
    plt.plot(data_time,data_pos[1,:])
    plt.plot(data_time,np.array([target_pos[1]]*np.size(data_time)),'r--')
    plt.xlabel('time[s]')
    plt.ylabel('angle[rad]')
    # plt.legend(('joint 2','ref'))
    plt.grid(True)
    # plt.ylim(-0.2,0.7)

    plt.subplot(313)
    plt.plot(data_time,data_pos[2,:])
    plt.plot(data_time,np.array([target_pos[2]]*np.size(data_time)),'r--')
    plt.xlabel('time[s]')
    plt.ylabel('angle[rad]')
    # plt.legend(('joint 3','ref'))
    plt.grid(True)
    # plt.ylim(-0.2,0.7)


def jointTor(fig_num,data_time,data_tor):
    plt.figure(fig_num)
    plt.subplot(311)
    plt.plot(data_time,data_tor[0,:])
    plt.xlabel('time[s]')
    plt.ylabel('joint 1[Nm]')
    # plt.legend(('joint 1','ref'))
    plt.grid(True)
    #plt.ylim(-0.2,0.7)
    plt.title('Joint Input Torque')

    plt.subplot(312)
    plt.plot(data_time,data_tor[1,:])
    plt.xlabel('time[s]')
    plt.ylabel('joint 2[Nm]')
    plt.grid(True)
    # plt.ylim(-0.2,0.7)

    plt.subplot(313)
    plt.plot(data_time,data_tor[2,:])
    plt.xlabel('time[s]')
    plt.ylabel('joint 3[Nm]')
    plt.grid(True)
    # plt.ylim(-0.2,0.7)

def eePos(fig_num,data_time,data_pos,desired_pos):
    plt.figure(fig_num)
    plt.subplot(311)
    plt.plot(data_time,data_pos[0,:])
    plt.plot(data_time,np.array([desired_pos[0]]*np.size(data_time)),'r--')
    plt.xlabel('time[s]')
    plt.ylabel('x[m]')
    plt.legend('x,x_d')
    plt.grid(True)
    plt.title('Cartesian Position Control')

    plt.subplot(312)
    plt.plot(data_time,data_pos[1,:])
    plt.plot(data_time,np.array([desired_pos[1]]*np.size(data_time)),'r--')
    plt.xlabel('time[s]')
    plt.ylabel('y[m]')
    plt.legend('y,y_d')

    plt.grid(True)

    plt.subplot(313)
    plt.plot(data_time,data_pos[2,:])
    plt.plot(data_time,np.array([desired_pos[2]]*np.size(data_time)),'r--')
    plt.xlabel('time[s]')
    plt.ylabel('z[m]')
    plt.legend('z,z_d')

    plt.grid(True)

def externalTorque(fig_num,data_time,ext_force):
    plt.figure(fig_num)
    plt.subplot(311)
    plt.plot(data_time,ext_force[0,:])
    plt.xlabel('time[s]')
    plt.ylabel('Force_x[N]')
    plt.grid(True)
    plt.title('External Force')

    plt.subplot(312)
    plt.plot(data_time,ext_force[1,:])
    plt.xlabel('time[s]')
    plt.ylabel('Force_y[N]')

    plt.grid(True)

    plt.subplot(313)
    plt.plot(data_time,ext_force[2,:])
    plt.xlabel('time[s]')
    plt.ylabel('Force_z[N]')

    plt.grid(True)