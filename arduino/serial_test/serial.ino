int num_1;   
int num_2;
int num_3;
int num_4;
int nLength;   //변수 선언

int get_int(char arr[],int a, int b){
  String m="";
  int i=a;
  int num;
  for(i;i<b;i++){ 
     m+=arr[i];
     }
     num=m.toInt();
     return num;
     
}

void setup()
{
  Serial.begin(9600); //baud rate 9600설정
  pinMode(13,OUTPUT);

}

void loop()
{ 
  

  char str[15]={0}; 
  num_1=0;
  num_2=0;
  num_3=0;
  num_4=0; //변수 초기화



  if(Serial.available()>0) //신호를 받았을 경우
   {

     nLength=Serial.readBytes(str,15); //할당된 길이만큼 데이터를 배열(str)에 저장 예시: x123y456z789c0
     //Serial.println(str);
     //Serial.println(nLength);

     //Serial.write(str);

     num_1=get_int(str,1,4);//str배열에 1부터 3까지 슬라이싱 후 int로 저장 예시=123
     num_2=get_int(str,5,8);//str배열에 5부터 7까지 슬라이싱 후 int로 저장 예시=456
     num_3=get_int(str,9,12);//str배열에 9부터 11까지 슬라이싱 후 int로 저장 예시=789
     num_4=get_int(str,13,14);//str배열에 13부터 3까지 슬라이싱 후 int로 저장 예시=0


     Serial.println(num_1);
     Serial.println(num_2); 
     Serial.println(num_3);
     Serial.println(num_4);
     
   }
    if(num_1==123 and num_2==456 and num_3==789 and num_4==0){
      digitalWrite(13,HIGH);
      delay(500);
      digitalWrite(13,LOW);
      delay(500);
      digitalWrite(13,HIGH);
      delay(500);
      digitalWrite(13,LOW);
      delay(500);
      digitalWrite(13,HIGH);
      delay(500);
      digitalWrite(13,LOW);


     

}

}
