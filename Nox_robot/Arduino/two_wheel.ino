#include <ros.h>
#include <ros/time.h>
#include <ArduinoHardware.h>
#include <geometry_msgs/Twist.h>
#include <MPU6050.h>
#include <std_msgs/Int32MultiArray.h>

#include <Wire.h> 


int enco_A = 2;
int enco_B = 3;

int enco1_A = 19;
int enco1_B = 18;

int dem = 0;
int dem1 = 0;

int aLastState_l;

int aLastState_r;

int timecho = 250;
unsigned long thoigian;
unsigned long hientai;

void dem_xung()
{
 
  int aState = digitalRead(enco_A); // Đọc trạng thái hiện tại của outputA
  int bState = digitalRead(enco_B); // Đọc trạng thái hiện tại của outputB

  // Xác định hướng quay
  if (aState != aLastState_l) { // Nếu có sự thay đổi
    if (bState != aState) {
        dem++; //đếm xung    
     } 
     else {
      dem--;
    }
  }

  aLastState_l = aState; // Cập nhật trạng thái trước của outputA
}
void dem_xung1()
{
  int aState = digitalRead(enco1_A); // Đọc trạng thái hiện tại của outputA
  int bState = digitalRead(enco1_B); // Đọc trạng thái hiện tại của outputB

  // Xác định hướng quay
  if (aState != aLastState_r) { // Nếu có sự thay đổi
    if (bState != aState) {
        dem1++; //đếm xung    
     } 
     else {
      dem1--;
    }
  }

  aLastState_r = aState; // Cập nhật trạng thái trước của outputA
}


int rpm = 0;
float tocdo = 0;
int rpm1 = 0;
float tocdo1 = 0;


int gocquaytru = 100;
int thangtru = 170;
int lechtru = 190;
const int Pin1 = 9;
const int Pin2 = 8;
const int EnableA = 6;


const int Pin3 = 12;
const int Pin4 = 10;
const int EnableB = 7;

float linear_vel = 0.0;
float angular_vel = 0.0;

MPU6050 mpu;

double w_r=0, w_l=0;

double wheel_rad = 0.06, wheel_sep = 0.26;


ros::NodeHandle  nh;
geometry_msgs::Twist msg;




void messageCb( const geometry_msgs::Twist& msg){
  linear_vel = msg.linear.x;
angular_vel = msg.angular.z;
  w_r = linear_vel - angular_vel/2.0;
  w_l = linear_vel + angular_vel/2.0;
   
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

std_msgs::Int32MultiArray encoder_msg;
ros::Publisher encoder_pub("encoder", &encoder_msg);

void setup()
{
  Serial.begin(9600);

  Serial.println("Initialize heeh");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
     Serial.println("Initialize");
    delay(500);
  }
  aLastState_l = digitalRead(enco_A);
  aLastState_r = digitalRead(enco1_A);
mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
   mpu.setGyroOffsetX(0);
   mpu.setGyroOffsetY(0);
   mpu.setGyroOffsetZ(0);
  

  mpu.calibrateGyro();

  mpu.setThreshold(0);
  Motors_init();
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(encoder_pub);
  
  pinMode(enco_A, INPUT);
   pinMode(enco_B, INPUT);
  attachInterrupt(0, dem_xung, CHANGE); //Ngắt ngoài

  pinMode(enco1_A, INPUT);
  pinMode(enco1_B, INPUT);
  attachInterrupt(4, dem_xung1, CHANGE); //Ngắt ngoài
 
  
}
void loop()
{  
  
  thoigian = millis();
   if (thoigian - hientai >= 100) //millis thay delay
  {
    hientai = thoigian;
    
    int32_t encoder_count[] ={dem,dem1};
    encoder_msg.data_length = 2;
    encoder_msg.data =encoder_count;
    encoder_pub.publish(&encoder_msg);
    dem = 0;
    dem1 = 0;

  }

  
  Vector normGyro = mpu.readNormalizeGyro();
  Serial.println(normGyro.XAxis);
  MotorL(w_l*1000,normGyro.XAxis);
  MotorR(w_r*1000,normGyro.XAxis);

  
  nh.spinOnce();
  delay(15);
}

void Motors_init(){

 pinMode(Pin1, OUTPUT);

 pinMode(Pin2, OUTPUT);

 pinMode(Pin3, OUTPUT);

 pinMode(Pin4, OUTPUT);

 pinMode(EnableA, OUTPUT);

 pinMode(EnableB, OUTPUT);
 digitalWrite(EnableA,HIGH);
 digitalWrite(EnableB,HIGH);

}
void thang()
{
   analogWrite(EnableA, 130);
     
   digitalWrite(Pin1, HIGH);
    
   digitalWrite(Pin2, LOW);
   analogWrite(EnableB, 130);
     
   digitalWrite(Pin3, HIGH);
    
   digitalWrite(Pin4, LOW);
}
void MotorL(double speed_L,double angular){


 if (speed_L > 0){
  if (angular_vel!=0)
     {
        analogWrite(EnableA, 130+18);
     
         digitalWrite(Pin1, HIGH);
    
         digitalWrite(Pin2, LOW);
     }
  
  else if(angular_vel==0)
  {
        if(angular==0)
      {
          analogWrite(EnableA, 130+18);
     
         digitalWrite(Pin1, HIGH);
    
         digitalWrite(Pin2, LOW);
      }
      else if(angular>0)
      {
        analogWrite(EnableA, 130+18);
     
         digitalWrite(Pin1, HIGH);
    
         digitalWrite(Pin2, LOW);
      }
      else if(angular<0)
      {
        analogWrite(EnableA, 110+18);
     
         digitalWrite(Pin1, HIGH);
    
         digitalWrite(Pin2, LOW);
      }
  }
 }

 else if (speed_L < 0){

//     Pulse_Width1=abs(Pulse_Width1);
     speed_L = abs(speed_L);
     if (angular_vel!=0)
     {
        analogWrite(EnableA, 130+18);
     
         digitalWrite(Pin1, LOW);
    
         digitalWrite(Pin2, HIGH);
     }
     else if(angular_vel==0)
     {
          if(angular==0)
          {
          analogWrite(EnableA, 130);
     
         digitalWrite(Pin1, LOW);
    
         digitalWrite(Pin2, HIGH);
         }
          else if(angular<0)
          {
        analogWrite(EnableA, 110+18);
     
         digitalWrite(Pin1, LOW);
    
         digitalWrite(Pin2, HIGH);
        }
        else if(angular>0)
        {
        analogWrite(EnableA, 110+18);
     
         digitalWrite(Pin1, LOW);
    
         digitalWrite(Pin2, HIGH);
         }
    }
       
  
 }

 else if (speed_L == 0){

     analogWrite(EnableA, 0);

     digitalWrite(Pin1, LOW);

     digitalWrite(Pin2, LOW);

 }

}


void MotorR(double speed_r,double angular){
  
 if (speed_r > 0){
   if (angular_vel!=0)
     {
         analogWrite(EnableB,130);
    
         digitalWrite(Pin3, HIGH);
    
         digitalWrite(Pin4, LOW);
     }
  
   else if(angular_vel==0)
   {
        if(angular==0)
        {
            analogWrite(EnableB,130);
           digitalWrite(Pin3, HIGH);
           digitalWrite(Pin4, LOW);
        }
        else if(angular<0)
       {
         analogWrite(EnableB,130);
    
         digitalWrite(Pin3, HIGH);
    
         digitalWrite(Pin4, LOW);
      }
      
      else if(angular>0)
      {
         analogWrite(EnableB,110);
    
         digitalWrite(Pin3, HIGH);
    
         digitalWrite(Pin4, LOW);
      }
      
   }
  
 }

 else if (speed_r < 0){
     speed_r=abs(speed_r);
      if (angular_vel!=0)
     {
      analogWrite(EnableB,130);
    
         digitalWrite(Pin3, LOW);
    
         digitalWrite(Pin4, HIGH);
     }
     else if(angular_vel==0)
     {
          if(angular==0)
          {
              analogWrite(EnableB,130);
        
             digitalWrite(Pin3, LOW);
        
             digitalWrite(Pin4, HIGH);
          }
        else if(angular>0)
        {
           analogWrite(EnableB,130);
           digitalWrite(Pin3, LOW);
      
           digitalWrite(Pin4, HIGH);
        }
        
        else if(angular<0)
        {
           analogWrite(EnableB,110);
           digitalWrite(Pin3, LOW);
      
           digitalWrite(Pin4, HIGH);
        }
     }
 }

 else if (speed_r == 0){

     analogWrite(EnableB,0);

     digitalWrite(Pin3, LOW);

     digitalWrite(Pin4, LOW);

 }

}
