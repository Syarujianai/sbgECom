/*
  Created by WeiZhang on 17-7-27
*/
#include "OdomUpdate.h"
#include <iostream>
using namespace std;


void Odometry::GetOdometry(long int WheelDistance1[4],long int WheelDir1[4], int state, double gyro)
{
  //void *memcpy(void *dest, const void *src, size_t n);
  memcpy(WheelDistance,WheelDistance1,4*sizeof(long int));
  memcpy(WheelDir,WheelDir1,4*sizeof(long int));
  State = state;//implict data-GYRO update for choose the car's mode
  GYRO = gyro;//implict data-GYRO update
}
//Louis 
void Odometry::GetWheelCount(long int WheelCountl[4])
{
  memcpy(WheelCount,WheelCountl,4*sizeof(long int));
}

Odom Odometry::CircleUpdate()//原地模式下的位移跟新

{//原地模式下，只有thea变化，x，y不变
  Odom modometry= {0.0,0.0,0.0};

  long int thea1 = (WheelDistance[0]-WheelCount[0])-(WheelDistance[1]-WheelCount[1])-\
                  (WheelDistance[2]-WheelCount[2])+(WheelDistance[3]-WheelCount[3]);

  modometry.thea = 3.14159*thea1*0.25*Scale/R;
  
  return modometry;

 }

//thea1逆时针为正，顺时针为负
Odom Odometry::NormalUpdate()//正常模式下的位移跟新
{//正常模式下的推算，前轮转向，后轮不转向

  Odom modometry= {0.0,0.0,0.0};
  
  double thea1 = 0;
  if(WheelDir[0] < 0 && WheelDir[1] < 0)//向左转
  {
    float Temp_R[4]={0};
    //cout << " left turn" << endl;
    Temp_R[1] = L/sin(-WheelDir[1]*AngleScale);//利用左前轮转角计算其对应的R1
    Temp_R[2] = L/tan(-WheelDir[1]*AngleScale);//利用左前轮转角计算其左后轮对应的R2   
    int temp=0; //记录delta距离有几个变化为0
    for(int i=0;i<4;i++)
      {
        if(i==1||i==2){thea1 += (WheelDistance[i]-WheelCount[i])/Temp_R[i];}//i==1||i==2?
        if(WheelDistance[i] == WheelCount[i])temp++;
        //cout << thea1 << " " << temp << " " << WheelDistance[i] << " " << WheelCount[i] << endl;
      }
    //if(temp>=3)return modometry;//如果右3个及以上轮子运动为零，那么直接输出delta距离为零
    //modometry.thea = 0.98 * (thea1*Scale*0.5) + 0.02 * GYRO;
    modometry.thea = thea1*Scale*0.5;
    modometry.x    = (Temp_R[2]+0.5*W)*(cos(modometry.thea)-1)-0.5*L*sin(modometry.thea);
    modometry.y    = (Temp_R[2]+0.5*W)*sin(modometry.thea)+0.5*L*(cos(modometry.thea)-1);
    //cout<<"左转R= "<<sqrt((Temp_R[2]+0.5*W)*(Temp_R[2]+0.5*W)+0.25*L*L)<<' '<<Temp_R[1]<<' '<<Temp_R[2]<<' '<<modometry.thea << " " << thea1 <<endl << endl;
  }
  else if(WheelDir[0] > 0 && WheelDir[1] > 0)//向右转
  {
    float Temp_R[4]={0};
    //cout << "right turn" << endl;
    Temp_R[0] = L/sin(WheelDir[0]*AngleScale);//利用右前轮转角计算其对应的R0
    Temp_R[3] = L/tan(WheelDir[0]*AngleScale);//利用右前轮转角计算右后轮对应的R3
    //cout << "Temp_R[0]" << Temp_R[0] << " " << Temp_R[3] << endl;
    int temp=0; //记录delta距离有几个变化为0
    for(int i=0;i<4;i++)
        { 
          if(i==0||i==3){thea1-= (WheelDistance[i]-WheelCount[i])/Temp_R[i];}

        if(WheelDistance[i] == WheelCount[i])temp++;
        //cout << thea1 << " " << temp << " " << WheelDistance[i] << " " << WheelCount[i] << endl;
        }
    //if(temp>=3)return modometry;//如果右3个及以上轮子运动为零，那么直接输出delta距离为零
      //modometry.thea = 0.98 * (thea1*Scale*0.5) + 0.02 * GYRO;
    modometry.thea = thea1*Scale*0.5;
    //下面的公式是以运动圆心为坐标原点建立的x y轴，然后在这个坐标下对质心 （Temp_R[1]-0.5*W ，L/2）旋转thea角度
    //利用公式一的方法求出变换后的质心，减去原来的质心就是在当前坐标系下的位移。
    //这里我的thea角度的正负还有仔细验证，具体还要根据数据辨别顺时针，逆时针的正负，也就是区分左右转弯。
    modometry.x    = -(Temp_R[3]+0.5*W)*(cos(modometry.thea)-1)-0.5*L*sin(modometry.thea);
    modometry.y    = -(Temp_R[3]+0.5*W)*sin(modometry.thea)+0.5*L*(cos(modometry.thea)-1);
    //cout<<"右转R= "<<sqrt((Temp_R[3]+0.5*W)*(Temp_R[3]+0.5*W)+0.25*L*L)<<' '<<WheelDir[0]*AngleScale<<' '<<modometry.thea << " " << thea1 <<endl << endl;
  }
  else//直行
  {
    int temp=0; //记录delta距离有几个变化为0
    long int distance=0;
    for(int i=0;i<4;i++)
    {
      distance+= (WheelDistance[i]-WheelCount[i]);
      //cout << "distance: " << distance << endl;
      //cout << WheelDistance[i] << " " << WheelCount[i] << endl;
      //cout << " distance: " << distance << endl;
      if(WheelDistance[i] == WheelCount[i])temp++;
    }
    //if(temp>=3)return modometry;//如果右3个及以上轮子运动为零，那么直接输出delta距离为零
        //cout<<"直行= "<<0.25*distance*Scale<<"  ";
    if(distance < 10)
      distance=0;
    modometry.y  = 0.25*distance*Scale;
  }
  return modometry;//return delta odom
}

Odom Odometry::StrightUpdate()//直线模式下的位移跟新
{
  Odom modometry= {0.0,0.0,0.0};

  long int distance = (WheelDistance[0]-WheelCount[0])+(WheelDistance[1]-WheelCount[1])+\
                  (WheelDistance[2]-WheelCount[2])+(WheelDistance[3]-WheelCount[3]);
  
  modometry.y    = 0.25*distance*Scale;
  return modometry;

}

Odom Odometry::Update()//根据状态位选择模式
{
  Odom modometry;
  switch(State)
  {
    case Circle :modometry= CircleUpdate(); break;
    case Normal :modometry= NormalUpdate(); break;
    case Stright :modometry=StrightUpdate();break;
  }
  return modometry;//return car's attitude
}

Odom Odometry::PositionInWorld(Odom &modometry,Odom  Temp_odometry)//跟新位置和方位角，这里是全局的坐标，以初始坐标，方向为0
{
  /*公式一
    | X    |     |cos(thea)  sin(thea)   0 | | x |
    | Y    | + = | -sin(thea) cos(thea)  0 |*| y |
    | Thea |     |  0          0         1 | | thea |
    
  */
  modometry.x += cos(modometry.thea)*Temp_odometry.x-sin(modometry.thea)*Temp_odometry.y;
  modometry.y += sin(modometry.thea)*Temp_odometry.x+cos(modometry.thea)*Temp_odometry.y;
  modometry.thea += Temp_odometry.thea;

}
