
// ---------------- Libraries

#include <ros.h>
#include <string.h>
#include <std_msgs/Float32.h>


// ---------------- Define

#define stepPin 9
#define dirPin 8
#define interruptPin 2

#define m0Pin 7
#define m1Pin 6
#define m2Pin 5


// ---------------- MicroStep Setup

bool m0 = HIGH;
bool m1 = HIGH;
bool m2 = LOW; // 1/8 step


// ---------------- Variables
 
int fpwm = 1; //Hz
int arr = 0;
volatile float vel = -5.5; // mm/s
float pos = 0; // mm
bool dir = HIGH;  

void platDirection(float vel){
  if (vel >= 0){
    dir = HIGH;
    }
  else if (vel < 0){
    dir = LOW;
    }
    
    digitalWrite(dirPin, dir);
  }


// ---------------- ROS Setup
// Activate Arduino rosserial --- rosrun rosserial_python serial_node.py /dev/ttyACM0

ros::NodeHandle node_handle;

std_msgs::Float32 msg;
std_msgs::Float32 ret_msg;

void callback(const std_msgs::Float32& msg) {
  
  vel = msg.data*10; //1/10 scale for RViz in mm    
}

ros::Subscriber<std_msgs::Float32> velocitySubscriber("prism_joint_vel", &callback);

ros::Publisher velocityPublisher("prism_joint_stepper", &ret_msg);


// ---------------- Model

float microStep = 1.0/8.0;
float angleStep = 1.8 * microStep;
float tPass = 360.0/angleStep;
float load = 2.0; //screw char. 2mm per rev

float resolution = load / tPass;

void microStep_velocity(float vel, float* resolution){
  float microStep = 1.0/8.0;

  if (abs(vel) <= 0.5){
    microStep = 1.0/32.0;
    m0 = HIGH;
    m1 = LOW;
    m2 = HIGH; // 1/32 step
    }
  
  else if (abs(vel) > 0.5 and abs(vel) <= 1){
    microStep = 1.0/16.0;
    m0 = LOW;
    m1 = LOW;
    m2 = HIGH; // 1/16 step
    }
  else if(abs(vel) > 1 and abs(vel) <= 12.5){
    microStep = 1.0/8.0;
    m0 = HIGH;
    m1 = HIGH;
    m2 = LOW; // 1/8 step
    }

    else if(abs(vel) > 12.5 and abs(vel) <= 14){ //FRICCIÓN
    microStep = 1.0/4.0;
    m0 = LOW;
    m1 = HIGH;
    m2 = LOW; // 1/2 step
    }

    else if(abs(vel) > 14 and abs(vel) <= 20){ //FRICCIÓN
    microStep = 1.0/2.0;
    m0 = HIGH;
    m1 = LOW;
    m2 = LOW; // 
    }

    else if(abs(vel) < 20){ //FRICCIÓN
    microStep = 1.0;
    m0 = LOW;
    m1 = LOW;
    m2 = LOW; // 
    }
    
    float angleStep = 1.8 * microStep;
    float tPass = 360.0/angleStep;
    float load = 2.0; 
    *resolution = load / tPass;
    
    digitalWrite(m0Pin, m0);
    digitalWrite(m1Pin, m1);
    digitalWrite(m2Pin, m2);
    }

    
// ---------------- Clock
long fclkIO = 8e6;
int prescaler = 8;
long fclk_pres = fclkIO/prescaler;

long calcARR(long fclk, float vel, float res, int* f_point){
  *f_point = int(abs(vel)/resolution);
  unsigned int ARR = fclk / fpwm;  
  return ARR;
  }


// ---------------- Time Variables
int startTime = 0;
int currentTime = 0;


// ---------------- Interrupts

int intActiv = 0;

ISR(TIMER1_OVF_vect)
{
  OCR1A = arr;
}


void setup() {

  // ---------------- ROS Setup
  
  node_handle.initNode();
  node_handle.subscribe(velocitySubscriber);
  node_handle.advertise(velocityPublisher);
  

  // ---------------- PIN Setup
  
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(m0Pin, OUTPUT);
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(interruptPin, INPUT);
  
  
  // ---------------- Platform Setup
  platDirection(vel);
  
  
  // ---------------- MicroStep Setup
  microStep_velocity(vel, &resolution);

  
  //---------------- Initial Velocity
  arr = calcARR(fclk_pres,vel,resolution,&fpwm);
  
  
  //---------------- Timer 1 16-bits Setup
  TCCR1A = _BV(COM1A0) | _BV(COM1B0) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) |_BV(WGM12) | _BV(CS11); // 1/8

  TIMSK1 = _BV(TOIE1); //Overflow Interrupt EN
  
  OCR1A = arr;
  
}

void loop() {

  // ---------------- Calculate ARR

  if(abs(vel) <= 0.1 and abs(vel) >= 0.05){
    vel = 0.1;
    }
  else if(abs(vel) < 0.05){
    vel = 0.0;
    }
  
  platDirection(vel);
  microStep_velocity(vel, &resolution);
  arr = calcARR(fclk_pres,vel,resolution,&fpwm);
  

  // ---------------- ROS Spin

  ret_msg.data = vel;
  velocityPublisher.publish(&ret_msg);
  
  node_handle.spinOnce();
  delay(10); // 10Hz publish rate
    
}
