#include "Arduino.h"
#include "MyCytronMotor.h"
#include <CytronMotorDriver.h>
#define INF 1e9

CytronMD right_motor(PWM_PWM, 6, 7);
CytronMD left_motor(PWM_PWM, 8, 9);

MyCytronMotor::MyCytronMotor(int _val, double _radius, double _tread){
  radius = _radius;
  tread  = _tread;
  val    = _val;
  sum    = 0;
  pastgap = 0;

  rightsum = 0;
  leftsum = 0;
  pastrightspeed = 0;
  pastleftspeed = 0;

  past_right_target_cnt = 0;
  past_left_target_cnt = 0;

  pgain = 1.0;
  igain = 0.03;
  dgain = 0.5;
  /* dgain = 0.0; */

  sumpgain = 1.5;
  /* sumigain = 0.005; */
  sumigain = 0.000;

  cnt_par_round = 918;

  timer = millis();
  flag = false;
  is_first_time = true;
}

void MyCytronMotor::calc_speed(int rightcnt, int leftcnt, int speed){
  rightsum = constrain(rightsum+right_target_cnt-rightcnt, -2000, 2000);
  leftsum = constrain(leftsum+left_target_cnt-leftcnt, -2000, 2000);
  rightspeed = constrain((right_target_cnt-rightcnt)*pgain+rightsum*igain, -speed, speed);
  leftspeed =  constrain((left_target_cnt-leftcnt)*pgain+leftsum*igain, -speed, speed);

  /* rightspeed = constrain(rightspeed+(rightspeed-pastrightspeed)*dgain+manipulation, -speed, speed); */
  /* leftspeed = constrain(leftspeed+(leftspeed-pastleftspeed)*dgain-manipulation, -speed, speed); */
  rightspeed = rightspeed+(rightspeed-pastrightspeed)*dgain;
  leftspeed = leftspeed+(leftspeed-pastleftspeed)*dgain;

  if(rightcnt == right_target_cnt) rightspeed = 0;
  if(leftcnt == left_target_cnt) leftspeed = 0;

  pastrightspeed = rightspeed;
  pastleftspeed = leftspeed;
}

int MyCytronMotor::back(int rightcnt, int leftcnt, int speed, double dist){
  if(is_first_time){
    right_target_cnt += (double)cnt_par_round/2/PI/radius*dist;
    left_target_cnt += (double)cnt_par_round/2/PI/radius*dist;
    /* right_target_cnt += 918; */
    /* left_target_cnt += 918; */
    is_first_time = false;
  }

gap = (leftcnt-past_left_target_cnt)-(rightcnt-past_right_target_cnt);
sum = constrain(sum+gap, -2000, 2000);
  manipulation = gap*sumpgain+sum*sumigain;
  calc_speed(rightcnt, leftcnt, speed);
  rightspeed = constrain(rightspeed+manipulation, -speed, speed);
  leftspeed = constrain(leftspeed-manipulation, -speed, speed);

  right_motor.setSpeed(rightspeed);
  left_motor.setSpeed(leftspeed);

  if(rightcnt == right_target_cnt && leftcnt == left_target_cnt){
    rightsum = 0;
    leftsum = 0;
    sum = 0;
    if(timer-millis() >= 500){
      past_right_target_cnt = right_target_cnt;
      past_left_target_cnt = left_target_cnt;
      is_first_time = true;
      return 0;
    }
  }else{
    timer = millis();
  }

  return 1;
}
int MyCytronMotor::back(int rightcnt, int leftcnt, int speed){
  return back(rightcnt, leftcnt, speed, INF);
}
int MyCytronMotor::back(int rightcnt, int leftcnt) {
  return back(rightcnt, leftcnt, 255);
}

int MyCytronMotor::forward(int rightcnt, int leftcnt, int speed, double dist){
  if(is_first_time){
    right_target_cnt -= (double)cnt_par_round/2/PI/radius*dist;
    left_target_cnt -= (double)cnt_par_round/2/PI/radius*dist;
    /* right_target_cnt -= 918; */
    /* left_target_cnt -= 918; */
    is_first_time = false;
  }

  gap = (leftcnt-past_left_target_cnt)-(rightcnt-past_right_target_cnt);
sum = constrain(sum+gap, -2000, 2000);
  manipulation = gap*sumpgain+sum*sumigain;
  calc_speed(rightcnt, leftcnt, speed);
  rightspeed = constrain(rightspeed+manipulation, -speed, speed);
  leftspeed = constrain(leftspeed-manipulation, -speed, speed);

  right_motor.setSpeed(rightspeed);
  left_motor.setSpeed(leftspeed);

  if(rightcnt == right_target_cnt && leftcnt == left_target_cnt){
    rightsum = 0;
    leftsum = 0;
    sum = 0;
    if(timer-millis() >= 500){
      past_right_target_cnt = right_target_cnt;
      past_left_target_cnt = left_target_cnt;
      is_first_time = true;
      return 0;
    }
  }else{
    timer = millis();
  }

  return 1;
}
int MyCytronMotor::forward(int rightcnt, int leftcnt, int speed){
  return forward(rightcnt, leftcnt, speed, INF);
}
int MyCytronMotor::forward(int rightcnt, int leftcnt) {
  return forward(rightcnt, leftcnt, 255);
}

int MyCytronMotor::right_rotation(int rightcnt, int leftcnt, int speed, double rad){
  if(is_first_time){
    right_target_cnt -= (cnt_par_round*tread)/(720*radius)*rad;
    left_target_cnt += (cnt_par_round*tread)/(720*radius)*rad;
    /* right_target_cnt -= 918; */
    /* left_target_cnt += 918; */
    is_first_time = false;
  }

  gap = (leftcnt-past_left_target_cnt)-(past_right_target_cnt-rightcnt);
sum = constrain(sum+gap, -2000, 2000);
  manipulation = gap*sumpgain+sum*sumigain;
  calc_speed(rightcnt, leftcnt, speed);
  rightspeed = constrain(rightspeed-manipulation, -speed, speed);
  leftspeed = constrain(leftspeed-manipulation, -speed, speed);

  right_motor.setSpeed(rightspeed);
  left_motor.setSpeed(leftspeed);

  if(rightcnt == right_target_cnt && leftcnt == left_target_cnt){
    rightsum = 0;
    leftsum = 0;
    sum = 0;
    if(timer-millis() >= 500){
      past_right_target_cnt = right_target_cnt;
      past_left_target_cnt = left_target_cnt;
      is_first_time = true;
      return 0;
    }
  }else{
    timer = millis();
  }

  return 1;
}
int MyCytronMotor::right_rotation(int rightcnt, int leftcnt, int speed){
  return right_rotation(rightcnt, leftcnt, speed, INF);
}
int MyCytronMotor::right_rotation(int rightcnt, int leftcnt){
  return right_rotation(rightcnt, leftcnt, 255);
}

int MyCytronMotor::left_rotation(int rightcnt, int leftcnt, int speed, double rad){
  if(is_first_time){
    right_target_cnt += (cnt_par_round*tread)/(720*radius)*rad;
    left_target_cnt -= (cnt_par_round*tread)/(720*radius)*rad;
    /* right_target_cnt += 918; */
    /* left_target_cnt -= 918; */
    is_first_time = false;
  }

  gap = (leftcnt-past_left_target_cnt)-(past_right_target_cnt-rightcnt);
sum = constrain(sum+gap, -2000, 2000);
  manipulation = gap*sumpgain+sum*sumigain;
  calc_speed(rightcnt, leftcnt, speed);
  rightspeed = constrain(rightspeed-manipulation, -speed, speed);
  leftspeed = constrain(leftspeed-manipulation, -speed, speed);

  right_motor.setSpeed(rightspeed);
  left_motor.setSpeed(leftspeed);

  if(rightcnt == right_target_cnt && leftcnt == left_target_cnt){
    rightsum = 0;
    leftsum = 0;
    sum = 0;
    if(timer-millis() >= 500){
      past_right_target_cnt = right_target_cnt;
      past_left_target_cnt = left_target_cnt;
      is_first_time = true;
      return 0;
    }
  }else{
    timer = millis();
  }

  return 1;
}
int MyCytronMotor::left_rotation(int rightcnt, int leftcnt, int speed){
  return left_rotation(rightcnt, leftcnt, INF);
}
int MyCytronMotor::left_rotation(int rightcnt, int leftcnt){
  return left_rotation(rightcnt, leftcnt, 255);
}

void MyCytronMotor::brake() {
  right_motor.setSpeed(0);
  left_motor.setSpeed(0);
}
