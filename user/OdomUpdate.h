#ifndef ODOMUPDATE_H
#define ODOMUPDATE_H

#include <iostream>
#include <fstream> 
#include <math.h>  
#include <cstring>
#include <string>

//static long int WheelCount[4]={0};//这是一个霍尔计数器，记录了四个轮子的到这次为止的总位移，是一个矢量，正传+，反转-
                                  //上传的WheelDistance是绝对的，所以减去WheelCount，就是这次的增量。this is delta x!
const float L=0.50;     //机器人长，宽数据，单位米
const float W=0.48;
const float R=0.34655;  //长方形的外接圆半径
const double Scale = 0.00890118;   //这里的Scale表示一个霍尔脉冲代表的弧长,（D×PI/90）
const double AngleScale = 1.227185e-5;  //角度换算amplify

typedef struct Position
{
//public:
  //syaru-17-12-3:replace the constrution fuction
  Position()=default;
  Position(double init_x,double init_y,double init_heading):
    x(init_x),y(init_y),thea(init_heading){}
  double x=0.0,y=0.0;
  double thea=0.0;
}Odom;

class Odometry
{
public:
  //Odometry();
  //~Odometry();
  void GetOdometry(long int WheelDistance[4],long int WheelDir[4],int state, double gyro);
  //Louis addition of code
  void GetWheelCount(long int WheelCountl[4]);
  long int WheelCount[4];
  //Odom modometry;//位姿 x y thea
  long int WheelDistance[4];//轮子位移
  long int WheelDir[4];//轮子转向角
  int State;//模式状态
  double GYRO;
  Odom CircleUpdate();//原地模式下的位移跟新
  Odom NormalUpdate();//正常模式下的位移跟新
  Odom StrightUpdate();//直线模式下的位移跟新
  Odom Update();//根据状态位选择模式
  Odom PositionInWorld(Odom &modometry,Odom  Temp_odometry);//跟新全局坐标

  enum
  {
    Circle=0,
    Normal=1,
    Stright=2   
  };

};
#endif
