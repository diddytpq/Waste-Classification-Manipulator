import intelligent_robotics as ir
import sympy as sp
import numpy as np
import numpy
import math
sp.init_printing()

def location_path2motor(path_list):
    string = ""
    for i in range(len(path_list)):
        for j in range(0,6):   
            string += str(path_list[i][j])+','

    return string


def get_pos_Link_Robot(l1, l2, l3, th1, th2, th3):
    x = np.cos(th1) * (l2 * np.sin(th2) + l3 * np.sin(th2 + th3))
    y = np.sin(th1) * (l2 * np.sin(th2) + l3 * np.sin(th2+ th3))
    z = l1 + (l2 * np.cos(th2)) + (l3 * np.cos(th2 + th3))
    return np.array([x, y, z])

def Trapezoidal_Traj_Gen_Given_Amax_and_T(amax,T,dt):
    a = amax
    
    if a*math.pow(T,2) >= 4:     
        v = 0.5*(a*T - math.pow(a,0.5)*math.pow((a*math.pow(T,2)-4),0.5))
    else:
        return False, False            
    
    time = 0
    t_save = time
    traj_save = np.array([0,0,0])
    while T >= time:
        if time >= 0 and time <= (v/a):
            sddot = a
            sdot = a*time
            s = 0.5*a*math.pow(time,2)
            
        if time > (v/a) and time <= (T-v/a):
            sddot = 0
            sdot = v 
            s = v*time - 0.5*math.pow(v,2)/a
            
        if time > (T-v/a) and time <= T:
            sddot = -a
            sdot = a*(T-time)
            s = (2*a*v*T - 2*math.pow(v,2) - math.pow(a,2)*math.pow((time-T),2))/(2*a)
        
        t_save = np.vstack((t_save, time))
        traj_save = np.vstack((traj_save, np.array([s,sdot,sddot])))
        time += dt

    return t_save, traj_save

# 0~1범위의 trajectory를 기반으로 실질적인 Path를 생성
def Path_Gen(start,goal,traj):
    path = start + traj*(goal-start)
    return path    

def convert_radian_from_npi_to_pi(rad):
    ang = rad
    while math.fabs(ang) >= (np.pi):
        if ang>0:
            ang-=(np.pi*2)
        else:
            ang+=(np.pi*2)
    return ang

def numerical_IK(x_des,th_now,func_J):


    func_J1 = func_J
    epsilon = 1e-10
    i = 0
    theta = th_now
    err = x_des - get_pos_Link_Robot(0.127,0.128,0.175, theta[0], theta[1], theta[2])

    while (np.linalg.norm(err) > epsilon):
        J_now = func_J1(theta[0], theta[1], theta[2])
        m, n = J_now.shape
        if m > n:
            J_inv = np.linalg.inv((J_now.transpose() @ J_now)) @ J_now.transpose()
        if m < n:
            J_inv = J_now.transpose() @ np.linalg.inv((J_now @ J_now.transpose()))    
        if m==n:
            J_inv = np.linalg.inv(J_now)     
           
        
        theta = theta + J_inv @ err
        for j in range(len(theta)):
            theta[j] = convert_radian_from_npi_to_pi(theta[j])
        err = x_des - get_pos_Link_Robot(0.127,0.128,0.175, theta[0], theta[1], theta[2])
        i += 1
        
    return theta

def theta2str(theta_list):

    for i in range(len(theta_list)):

        theta_list[i][0] = int(theta_list[i][0])
        theta_list[i][1] = int(theta_list[i][1])
        theta_list[i][2] = int(theta_list[i][2])

        th_5 = 135 - (180 - (theta_list[i][1] + theta_list[i][2]))

        theta_list[i][0] = -(theta_list[i][0]) + 135
        theta_list[i][1] = -(theta_list[i][1]) + 135
        theta_list[i][2] = -(theta_list[i][2]) + 135

        theta_list[i].append('135')
        theta_list[i].append(str(th_5))
        theta_list[i].append(str(90))
        
        theta_list[i][0] = str(theta_list[i][0])
        theta_list[i][1] = str(theta_list[i][1])
        theta_list[i][2] = str(theta_list[i][2])  
 
    
    return theta_list
 

def inverse_path(location_list):

    #목표 좌표 설정
    #wrist_x = (95.5 + location_list[1] * 0.55) * 0.001
    wrist_x = (380.5 - location_list[1] * 0.55) * 0.001
    wrist_y = (location_list[0] * 0.55 -75) *0.001
    wrist_z = 235.5 * 0.001 



    time, traj = Trapezoidal_Traj_Gen_Given_Amax_and_T(1.5, 2, 0.01)
    x_traj = Path_Gen(0.175,wrist_x,traj[:,0])
    y_traj = Path_Gen(0,wrist_y,traj[:,0])
    z_traj = Path_Gen(0.235,wrist_z,traj[:,0])

    x_up = Path_Gen(wrist_x,wrist_x,traj[:,0])
    y_up = Path_Gen(wrist_y,wrist_y,traj[:,0])
    z_up = Path_Gen(wrist_z,wrist_z+0.08,traj[:,0])


    print("x = {} y = {} z = {}".format(wrist_x,wrist_y,wrist_z))

    #그리퍼의 중심부위치 205mm+베이스 높이 15mm


    theta = np.array([0.001,0.001,0.001])

    theta1, theta2, theta3 = sp.symbols('theta1, theta2, theta3')
    l1, l2, l3, x, y, z = sp.symbols('l1, l2, l3, x, y, z')
    T01 = ir.DH(0, 0, l1, theta1)
    T12 = ir.DH(0, -sp.pi / 2, 0, -sp.pi / 2 + theta2)
    T23 = ir.DH(l2, 0, 0, sp.pi/2 + theta3)
    T34 = ir.DH(0, sp.pi/2, l3, 0)
    T04 = sp.simplify(T01 * T12 * T23 * T34)

    theta_1, theta_2, theta_3 = ir.dynamicsymbols('theta_1, theta_2, theta_3')
    w_0_0 = sp.Matrix([[0], [0], [0]])
    w_1_1 = ir.get_angular_vel_R(T01, w_0_0, theta_1.diff())
    w_2_2 = ir.get_angular_vel_R(T12, w_1_1, theta_2.diff())
    w_3_3 = ir.get_angular_vel_R(T23, w_2_2, theta_3.diff())
    w_4_4 = ir.get_angular_vel_R(T34, w_3_3, 0)

    v_0_0 = sp.Matrix([[0], [0], [0]])
    v_1_1 = ir.get_linear_vel_R(T01, w_0_0, v_0_0)
    v_2_2 = ir.get_linear_vel_R(T12, w_1_1, v_1_1)
    v_3_3 = ir.get_linear_vel_R(T23, w_2_2, v_2_2)
    v_4_4 = ir.get_linear_vel_R(T34, w_3_3, v_3_3)

                                
    qd = sp.Matrix([[theta_1.diff()], [theta_2.diff()], [theta_3.diff()]])

    w_0_4 = ir.get_R_from_T(T04) * w_4_4
    v_0_4 = ir.get_R_from_T(T04) * v_4_4
    J_0_4 = ir.get_Jacobian_from_vel(w_0_4, v_0_4, qd)


    J = J_0_4[0:3, :].subs({l1:0.127, l2:0.128, l3:0.175})
    func_J = sp.lambdify([theta1, theta2, theta3], J, 'numpy')



    for index in range(len(time)):
        x_des = np.array([x_traj[index], y_traj[index], z_traj[index]])
        theta = numerical_IK(x_des,theta,func_J)
        if index == 0:
            theta_store = theta
        else:
            theta_store = np.vstack([theta_store, theta])

    for index in range(len(time)):
        x_des = np.array([x_up[index], y_up[index], z_up[index]])
        theta = numerical_IK(x_des,theta,func_J)
        if index == 0:
            theta_store_up = theta
        else:
            theta_store_up = np.vstack([theta_store_up, theta])    
    
    theta_traj = np.degrees(theta_store)
    theta_up = np.degrees(theta_store_up)

    theta_list = theta_traj.tolist()
    theta_up_list = theta_up.tolist()

    theta_list = theta2str(theta_list)
    theta_up_list = theta2str(theta_up_list)
        

    return theta_list, theta_up_list
        

if __name__ == "__main__":

    location_list = [136,350,1]
    #location_list = [136,20,467]
    theta, theta_up = inverse_path(location_list)
    #for i in range(len(theta)):
    #    print(theta_up[i])
    print('1,' + location_path2motor(theta))
    print('2,' + location_path2motor(theta_up))