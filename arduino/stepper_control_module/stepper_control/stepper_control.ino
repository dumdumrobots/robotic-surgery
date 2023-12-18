// --- Libraries

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

// --- Time Variables

volatile float current_time;
float delta_time;

volatile float start_time;
volatile float diff_time;

// --- Time Variables

uint32_t delta_freq;
uint32_t pwm_freq;

uint32_t sys_clk_freq = 16e6;
uint16_t t0_prescaler = 1024;
uint16_t t1_prescaler = 64;


// --- Stepper Variables

uint8_t dir;

volatile float micro_step = 1;
volatile float resolution = 0.01; //mm per pulse

// --- Model Variables

volatile float goal_velocity;  //mm per sec
//volatile float current_linear_vel;

volatile float goal_position; //mm
volatile float current_position; //mm

volatile float error_position; //mm

// --- Timer TOPs

uint8_t T0_TOP;
uint16_t T1_TOP;

volatile uint8_t onPosition = 0;
volatile int8_t coherent_dir = 0;


// --- ROS Setup 
// Activate Arduino rosserial --- rosrun rosserial_python serial_node.py /dev/ttyACM0

ros::NodeHandle node_handle;

std_msgs::Float32 goal_pos_callback_msg;
std_msgs::Float32 goal_vel_callback_msg;
std_msgs::Float32 current_pos_feedback_msg;


void pos_callback(const std_msgs::Float32& goal_pos_callback_msg) {
  goal_position = goal_pos_callback_msg.data;
}

void vel_callback(const std_msgs::Float32& goal_vel_callback_msg) {
  goal_velocity = goal_vel_callback_msg.data; 
}

ros::Subscriber<std_msgs::Float32> goal_pos_subscriber("prism_joint__goal_position", &pos_callback);
ros::Subscriber<std_msgs::Float32> goal_vel_subscriber("prism_joint__goal_velocity", &vel_callback);

ros::Publisher pos_feedback_publisher("prism_joint__current_position", &current_pos_feedback_msg);


void setup(){
  
  // --- ROS Init
  node_handle.initNode();
  node_handle.subscribe(goal_pos_subscriber);
  node_handle.subscribe(goal_vel_subscriber);
  node_handle.advertise(pos_feedback_publisher);
  

  //PIN I/O Setup
  DDRB = 0;
  DDRB |= _BV(PB2);
  DDRB |= _BV(PB1);

  DDRD = 0;
  DDRD &= ~_BV(PD2);
  DDRD |= _BV(PD5);
  DDRD |= _BV(PD6);
  DDRD |= _BV(PD7);

  PORTD = 0;
  PORTD &= ~_BV(PD2);

  
  //External Interrupt INT0
  EICRA |= _BV(ISC01);
  EICRA &= ~_BV(ISC00);

  EIMSK |= _BV(INT0);

  sei();


  //Initial Variables
  current_time = 0;
  
  goal_position = 0;
  error_position = 0;
  
  if((PIND & _BV(PD2)) != 0){
    current_position = 300;
    goal_velocity = -10;
  }
  else{
    current_position = 0;
    goal_velocity = 0;
  }


  //Timer0 Delta
  delta_freq = 1000;
  delta_time = 0.001;

  T0_TOP = (sys_clk_freq / (t0_prescaler * delta_freq)) - 1;

  timer0_setup(&T0_TOP);
  timer1_setup(&T1_TOP);

  delay_t0(1);
}

void loop(){

  error_position = current_position - goal_position;

  coherent_dir = -(error_position / abs(error_position)) * (goal_velocity/abs(goal_velocity));

  if (abs(error_position) >= resolution){
    onPosition = 0;
  }

  if (onPosition == 0 && coherent_dir == 1 && goal_position >= 0){
    stepper_microstep(&resolution, &micro_step);
    stepper_direction(goal_velocity, &dir);
    stepper_speed(&pwm_freq, &goal_velocity, &T1_TOP, &resolution);

    timer1_start();
  }

  current_pos_feedback_msg.data = current_position;
  pos_feedback_publisher.publish(&current_pos_feedback_msg);
  
  node_handle.spinOnce();
  delay_t0(0.01);
}


void stepper_direction(float lin_vel, uint8_t *dir){
  if(lin_vel >= 0){
    PORTB |= _BV(PB2);
    *dir = 1;
  }
  else{
    PORTB &= ~_BV(PB2);
    *dir = 0;
  }
}

void stepper_speed(uint32_t *pwm_freq, float *velocity, uint16_t *TOP, float *resolution){
  *pwm_freq = abs(*velocity) / *resolution;
  *TOP = (sys_clk_freq / (2 * t1_prescaler * *pwm_freq)) - 1;
}

void stepper_microstep(float *resolution, float *micro_step){
  *resolution = 0.01 * *micro_step;

  /*

  if (*velocity < 1){
    PORTD &= ~(_BV(PD5));
    PORTD &= ~(_BV(PD6));
    PORTD &= ~(_BV(PD7));
  }

  else if (*velocity < 1){
    //PORTD |= _BV(PD5);
    //PORTD |= _BV(PD6);
    //PORTD |= _BV(PD7);
  }

  else if (*velocity < 1){
    //PORTD |= _BV(PD5);
    //PORTD |= _BV(PD6);
    //PORTD |= _BV(PD7);
  }

  else if (*velocity < 1){
    //PORTD |= _BV(PD5);
    //PORTD |= _BV(PD6);
    //PORTD |= _BV(PD7);
  }

  else if (*velocity < 1){
    //PORTD |= _BV(PD5);
    //PORTD |= _BV(PD6);
    //PORTD |= _BV(PD7);
  }*/
}


void timer1_start(){
  TCCR1B |= _BV(CS11);
  TCCR1B |= _BV(CS10);
}

void timer1_stop(){
  TCCR1B &= ~_BV(CS11);
  TCCR1B &= ~_BV(CS10);
}

void timer1_setup(uint16_t *TOP){

  OCR1A = *TOP;

  TIMSK1 |= _BV(TOIE1);

  TCCR1A = 0;
  TCCR1A |= _BV(COM1A0);
  TCCR1A |= _BV(COM1B0);
  TCCR1A |= _BV(WGM10);
  TCCR1A |= _BV(WGM11);

  TCCR1B = 0;
  TCCR1B |= _BV(WGM12);
  TCCR1B |= _BV(WGM13);

  //TCCR1B |= _BV(CS11);
  //TCCR1B |= _BV(CS10);
}


void delay_t0(float delay_sec){
  start_time = current_time;
  diff_time = 0;

  while (diff_time <= delay_sec){
    diff_time = current_time - start_time;
  }
}

void timer0_setup(uint8_t *TOP){
  
  OCR0A = *TOP;

  TIMSK0 |= _BV(OCIE0A);

  TCCR0A = 0;
  TCCR0A |= _BV(WGM00);
  TCCR0A |= _BV(WGM01);

  TCCR0B = 0;
  TCCR0B |= _BV(WGM02);
  TCCR0B |= _BV(CS00);
  TCCR0B |= _BV(CS02);
}


ISR(INT0_vect){

  if(PD2)

  current_position = 0;

  goal_velocity = 0;
  goal_position = 0;

  error_position = 0;

  timer1_stop();
}


ISR(TIMER0_COMPA_vect){
  OCR0A = T0_TOP;
  current_time += delta_time;
}


ISR(TIMER1_OVF_vect){
  if(dir == 1){
    if (current_position >= goal_position){
      onPosition = 1;
      timer1_stop();
      return;
    }
    current_position += resolution/2;
  }
  
  else{
    if (current_position <= goal_position){
      onPosition = 1;
      timer1_stop();
      return;
    }
    current_position -= resolution/2;
  }

  OCR1A = T1_TOP;
}