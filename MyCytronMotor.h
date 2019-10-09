#ifndef _MyCytronMotor_h
#define _MyCytronMotor_h
#include "Arduino.h"
#include <CytronMotorDriver.h>

class MyCytronMotor {
  private:
    double pgain, igain, dgain;
    double sumpgain, sumigain;
    double radius, tread;
    int gap, pastgap;
    int sum, val;
    int manipulation;  // 操作量
    int rightspeed, leftspeed;
    int pastrightspeed, pastleftspeed;
    int rightsum, leftsum;
    int cnt_par_round; // 一周あたりのカウント数
    int right_target_cnt;    // 目標カウント数
    int left_target_cnt;    // 目標カウント数
    int past_right_target_cnt;    // 目標カウント数
    int past_left_target_cnt;    // 目標カウント数
    int timer;
    boolean flag;
    boolean is_first_time;
  public:
    MyCytronMotor(int _val, double _radius, double _tread);

    void calc_speed(int rightcnt, int leftcnt, int speed);
    void brake();

    int forward(int rightcnt, int leftcnt, int speed, double dist);
    int forward(int rightcnt, int leftcnt, int speed);
    int forward(int rightcnt, int leftcnt);

    int back(int rightcnt, int leftcnt, int speed, double dist);
    int back(int rightcnt, int leftcnt, int speed);
    int back(int rightcnt, int leftcnt);


    int right_rotation(int rightcnt, int leftcnt, int speed, double rad);
    int right_rotation(int rightcnt, int leftcnt, int speed);
    int right_rotation(int rightcnt, int leftcnt);

    int left_rotation(int rightcnt, int leftcnt, int speed, double rad);
    int left_rotation(int rightcnt, int leftcnt, int speed);
    int left_rotation(int rightcnt, int leftcnt);
};

#endif
