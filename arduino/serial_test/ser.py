
import serial
import time

ser = serial.Serial('/dev/ttyACM1', 9600)#serial open
#ser_2 = serial.Serial('COM7', 9600)

time.sleep(1)


while True:
    print("Input postion: ")
    string = input().split(' ')
    string = ''.join(string)
    if string=="break":
        break
    print(string)
    Trans=string.encode('utf-8') #문자를 8비트로 전환
    ser.write(Trans)  #연결된 포트로 비트 전송  
                      #예시 1 150 150 150 150 150 150 150
                      # 1 090 090 045 090 090 045 090
                      #1 135 135 45 135 045 045 090
                      #1 135 135 135 135 135 045 090
                      #1 125 125 125 125 125 125 125
                      #1 135 135 135 135 135 135 135
                      # 1 000 000 000 000 000 000 000 
                      # 1 180 180 180 180 180 180 180 
                      # 1 000 020 050 080 11 140 170

    while True:
        if ser.readable()>0:
            res = ser.readline()
            print("receive message :",res.decode()[:len(res)-1])
            break

ser.close() #serial close

