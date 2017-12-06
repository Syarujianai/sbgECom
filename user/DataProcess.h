#ifndef _DATA_PROCESS_H
#define _DATA_PROCESS_H

#include "control_car.h"//car class
#include "OdomUpdate.h"//odom struct and Odometry class
#include <sys/time.h>//struct timeval
//#include <math.h>//fabs()
#include <sbgEComLib.h>//imuData

#define  PI   3.14159265358979323846   //圆周率
//shift can store value in a list
//require:8 datas(0,1 wheel's angle,and 0,1,2,3 odometry sums 6 data),and design a mean filter,read in ubuntu's stamp
class DataProcess
{
public:
	
	DataProcess(double init_x,double init_y,double init_heading):
		trace(init_x,init_y,init_heading),thea_gyro(init_heading){}// 右手系数值 tanzby
	//~DataProcess();

	bool start_calc=false;

	struct timeval timeStampNow;
	struct timeval timeStampLast;

	Odom temp_li;//a template of Odom li 
	Odom li;//delta x,delta y,delta thea in relative frame 
	Odometry haha; 
	Odom trace;//car trace's record,which are all initialized by zero in absolute frame

	Odometry& update_odom(car &CarData,const SbgLogImuData &ImuData,int state,int mode);
	inline void previous_read(car &myCarData,const SbgLogImuData &myImuData);
	inline void else_read(car &myCarData,const SbgLogImuData &myImuData);
	inline void theaGyro_calc(void);

private:
	int nn=0,calc_counts=0;
	bool isInitOk=false;

	long delta_time=0;
	inline long ubuntu_time_show(){return 1000000L*timeStampNow.tv_sec+timeStampNow.tv_usec;}

	long int carDist[4]={0},carAngle[4]={0},carDistLast[4]={0};

    double gyroscopes=0.0,gyroscopes_last=0.0,delta_angle=0.0,gyro_shift=0.0;
    double thea_gyro=0.0;	
};

#endif
