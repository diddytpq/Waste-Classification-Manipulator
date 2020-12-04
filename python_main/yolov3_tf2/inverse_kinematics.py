#import intelligent_robotics as ir
import numpy as np
import cmath
import math
import sympy as sym

sym.init_printing()





def inverse_6(location_list):

    sth1,sth2,sth3,sth4,sth5,sth6=sym.symbols('th1,th2,th3,th4,th5,th6')

    #Link 길이 설정
    a1=127.7
    a3=175.42
    a4=97.02
    a2=128.02
    a5=76.15

    #좌표 설정
    wrist_x = 127.5 + location_list[1] * 0.55
    wrist_y = location_list[0] * 0.55 -75
    wrist_z = 215.5

    print("x = {} y = {} z = {}".format(wrist_x,wrist_y,wrist_z))

    #그리퍼의 중심부위치 205mm+베이스 높이 15mm

    ##th1 구하기
    th2 = []
    f_th2=0
    th1 = math.atan2(wrist_y, wrist_x)
    th1 = th1*180/np.pi

    f_th1 = th1

    p_2 = np.square(wrist_x)+np.square(wrist_y)
    sq_p_2 = np.sqrt(p_2)
    p = p_2+(np.square(wrist_z-a1))

    ##th3 구하기
    costh3 = (p-np.square(a2)-np.square(a3))/(2*a2*a3)

    if costh3 > 1 or costh3 <-1 :
        print ("불가능")
    else:
        sq_costh3 = np.square(costh3)
        k = 1-sq_costh3
        sinth3 = math.sqrt(k)
        th3 = math.atan2(sinth3,costh3)
        f_th3 = th3*180/np.pi
    ##th2 구하기
         
    #th2_1         
    costh_1 = -((np.square(a3)-np.square(a2)-p)/(2*a2*np.sqrt(p)))
            
    #th2_2
    costh_2 = sq_p_2/np.sqrt(p)
            
    total_costh = [costh_1,costh_2]    

    for i in total_costh:
        a = np.square(i)
        b = 1-a
        c = np.sqrt(b)
        d = math.atan2(c,i)
        e = d*180/np.pi
        th2.append(e)
        
    f_th2 = th2[0]+th2[1]

    f_th2 = 90 - f_th2

    #th5 구하기


    f_th5 = 180-(f_th2+f_th3)


    th_list = [f_th1,f_th2,f_th3,0,f_th5,f_th1]    

    
    #th_list = [int(f_th1+45),int(f_th2+45),int(f_th3+45), 0, 135]
    print(th_list)

    return th_list
        



 
if __name__ == "__main__":

    location_list = [136,240,1]
    inverse_6(location_list)