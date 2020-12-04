#include <Servo.h>
#include <Stepper.h>

Servo myservo_1;
Servo myservo_2;
Servo myservo_3;
Servo myservo_4;
Servo myservo_5;
Servo myservo_6;
Servo myservo_7;

const int stepsPerRevolution = 1600;

Stepper myStepper_1(stepsPerRevolution,6,7);

int driverPUL = 7;    // PUL- pin
int driverDIR = 6;    // DIR- pin
int driverENV = 5;    // ENV- pin

//변수 지정 칸//
int rail_power=1; //레일 작동 담당받음
int arm_power=0; //로봇팔 움직임 담당

int motor_1;   
int motor_2;
int motor_3;
int motor_4;
int motor_5;
int motor_6;
int motor_grip;


int motor_init_1 = 135;
int motor_init_2 = 140;
int motor_init_3 = 50; 
int motor_init_4 = 135; 
int motor_init_5 = 50; 
int motor_init_6 = 90; 

int drop_1;
int drop_2;
int drop_3;
int drop_4; 
int drop_5; 
int drop_6;  


int nLength;


int num_split(int num_list[1207],char str[5100], int len)
{
  //Serial.println(len);
  //Serial.println(str);
  //int (*num_list[1206]);
  String num_str = "";
  int i = 0 ;
  int j = 0 ;
  char a;
    for(i; i<len; i++)
    {
  
      a = str[i];
      //Serial.println(str[1]);
      if(a == ',')
      {
        //Serial.println(num_str);
        num_list[j] = num_str.toInt();
        num_str = "";
        j++;
        continue;
      }
  
      num_str += a;
    }
  //Serial.println(j);
  //Serial.println(num_list[1205]);
   return j;
}
 
void arm_moving(int _1, int _2, int _3, int _4, int _5, int _6)
{
  if(_1 >3 && _2 > 3 && _3  > 3 && _4  > 3 &&  _5  > 3)
  {
    myservo_1.write(_1/1.5);
    myservo_2.write(_2/1.5 + 3);      
    myservo_3.write(_3/1.5 + 3);      
    //myservo_4.write(_4/1.5);      
    myservo_5.write(_5/1.5 + 6);      
    myservo_6.write(_6);
    delay(2);
  }
}


void arm_move_setup(int _1, int _2, int _3, int _4, int _5, int _6)       //초기 세팅으로 이동
{
  while(1)
  {
    int pre_motor_1 = _1/ 1.5 - myservo_1.read();
    int pre_motor_2 = _2/ 1.5  - myservo_2.read();
    int pre_motor_3 = _3/ 1.5  - myservo_3.read();
    int pre_motor_4 = _4/ 1.5 - myservo_4.read();
    int pre_motor_5 = _5/ 1.5 - myservo_5.read();
    int pre_motor_6 = _6 - myservo_6.read();

  
      if(pre_motor_1>0)
      { 
        myservo_1.write(myservo_1.read()+1);
      }
      else if(pre_motor_1<0)
      {
        myservo_1.write(myservo_1.read()-1);
      }
      if(pre_motor_2>0)
      { 
        myservo_2.write(myservo_2.read()+1);
      }
      else if(pre_motor_2<0)
      {
        myservo_2.write(myservo_2.read()-1);
      }      
      if(pre_motor_3>0)
      { 
        myservo_3.write(myservo_3.read()+1);
      }
      else if(pre_motor_3<0)
      {
        myservo_3.write(myservo_3.read()-1);
      }      
      if(pre_motor_4>0)
      { 
        myservo_4.write(myservo_4.read()+1);
      }
      else if(pre_motor_4<0)
      {
        myservo_4.write(myservo_4.read()-1);
      }      
      if(pre_motor_5>0)
      { 
        myservo_5.write(myservo_5.read()+1);
      }
      else if(pre_motor_5<0)
      {
        myservo_5.write(myservo_5.read()-1);
      }      
      if(pre_motor_6>0)
      { 
        myservo_6.write(myservo_6.read()+1);
      }
      else if(pre_motor_6<0)
      {
        myservo_6.write(myservo_6.read()-1);
      }
    delay(50);
    
    if(pre_motor_1 == 0 && pre_motor_2 == 0 && pre_motor_3 == 0 && pre_motor_4 == 0 && pre_motor_5 == 0 && pre_motor_6 ==0)
    {
      break;
    }
    
  }
  
}










void arm_drop_pos() //캔 모으는곳으로 이동
{
    while(1)
  {
      int pre_motor_1 = 150 - myservo_1.read();
      if(pre_motor_1>0)
      { 
        myservo_1.write(myservo_1.read()+1);
      }
      else if(pre_motor_1<0)
      {
        myservo_1.write(myservo_1.read()-1);
      }
    delay(25);
    if(pre_motor_1==0)
    {
      break;
    }
  }
}


void gripper_move(int w, int t)
{
  myservo_7.write(w);
  delay(t);
  myservo_7.write(90);
  delay(500);
}

void rail_start()
{
  myStepper_1.step(200);
}

void setup() 
{
    Serial.begin(9600);

    pinMode(driverPUL, OUTPUT);
    pinMode(driverDIR, OUTPUT);
    pinMode(driverENV, OUTPUT);

    myStepper_1.setSpeed(200);

    digitalWrite(driverDIR, LOW);
    digitalWrite(driverPUL, HIGH);
    digitalWrite(driverENV,HIGH);
    
    myservo_1.attach(22);
    myservo_2.attach(24);
    myservo_3.attach(26);
    myservo_4.attach(28);
    myservo_5.attach(30);
    myservo_6.attach(23);
    myservo_7.attach(25); //6축 + 그리퍼 핀 설정

    arm_move_setup(135,135,135,135,135,90);       //초기 세팅으로 이동
    gripper_move(80,2300); // 그리퍼 초기 각도

    //Serial.println(myservo_1.read());
    //Serial.println(myservo_2.read());
    //Serial.println("ready");
}


void loop() 
{
    rail_start();
    //Serial.println(myservo_4.read());
    //Serial.println("rail move");
    if(Serial.available()>0)
    {

      while(1)
      {
        
        int num_list[1207] = {};
        char serial_buffer[5100] = {};
        int j;
        nLength = Serial.readBytes(serial_buffer,5100); 
        //Serial.println(nLength);
        //Serial.println(serial_buffer[0]);
        j = num_split(num_list,serial_buffer,nLength); //num_list[1206]
        //Serial.println(num_list[0]);



        if(num_list[0]==1) //G
        {
          
          arm_move_setup(myservo_1.read()*1.5,myservo_2.read()*1.5,myservo_3.read()*1.5,myservo_4.read()*1.5,myservo_5.read()*1.5,0);

          arm_move_setup(motor_init_1,motor_init_2,motor_init_3,motor_init_4,motor_init_5,0);
          delay(1000);
          //Serial.println(j);
          for(int i=0; i<201; i++)
          {
            int k = 6 * i;
                        
            /*Serial.println(num_list[1+0%6 +k]);
            Serial.println(num_list[1+1%6+k]);
            Serial.println(num_list[1+2%6 +k]);
            Serial.println(num_list[1+3%6 +k]);
            Serial.println(num_list[1+4%6 +k]);
            Serial.println(num_list[1+5%6 +k]);
            */
            
            //Serial.println(myservo_1.read());


            arm_moving(num_list[1+k],num_list[2+k],num_list[3+k],num_list[4+k],num_list[5+k],0);
            //arm_moving(num_list[1+0%6 +k],num_list[1+1%6 +k],num_list[1+2%6 +k],num_list[1+3%6 + k],num_list[1+4%6 + k],num_list[1+5%6 + k]);
          }
        

          

     

          delay(1000);
          gripper_move(110,2000);
          
          Serial.println("done");
        }

        if(num_list[0]==2) //S
        {
          arm_move_setup(motor_init_1,motor_init_2,motor_init_3,motor_init_4,motor_init_5,motor_init_6);

          //Serial.println(j);
          for(int i=0; i<201; i++)
          {
            int k = 6 * i;
            /*
            Serial.println(num_list[1+0%6 +k]);
            Serial.println(num_list[1+1%6+k]);
            Serial.println(num_list[1+2%6 +k]);
            Serial.println(num_list[1+3%6 +k]);
            Serial.println(num_list[1+4%6 +k]);
            Serial.println(num_list[1+5%6 +k]);
            */
            

            arm_moving(num_list[1+k],num_list[2+k],num_list[3+k],num_list[4+k],num_list[5+k],num_list[6+k]);
            //arm_moving(num_list[1+0%6 +k],num_list[1+1%6 +k],num_list[1+2%6 +k],num_list[1+3%6 + k],num_list[1+4%6 + k],num_list[1+5%6 + k]);
          }



          delay(1000);
          gripper_move(110,2000);
          
          Serial.println("done");
        }

        

        if(num_list[0]==3) //END
        {
           j = num_split(num_list,serial_buffer,nLength); //num_list[1206]
  
           for(int i=0; i<201; i++)
          {
            int k = 6 * i;
            /*
            Serial.println(num_list[1+0%6 +k]);
            Serial.println(num_list[1+1%6+k]);
            Serial.println(num_list[1+2%6 +k]);
            Serial.println(num_list[1+3%6 +k]);
            Serial.println(num_list[1+4%6 +k]);
            Serial.println(num_list[1+5%6 +k]);
            */
            arm_moving(num_list[1+0%6 +k],num_list[1+1%6 +k],num_list[1+2%6 +k],num_list[1+3%6 + k],num_list[1+4%6 + k],myservo_6.read());
          }
          delay(500);
          arm_drop_pos();
          
          delay(500);
          
          gripper_move(80,2300);
          
          arm_move_setup(135,135,135,135,135,90);
          
          delay(500);
          
          Serial.println("done_2"); //컴퓨터로 작업 종료 신호 보내기
          break;
          

        }

          
          //Serial.println(myservo_1.read());
          //Serial.println(myservo_2.read());

        }
          
     }

  }
    
