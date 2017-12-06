#include "DataProcess.h"

using namespace std;

void DataProcess::previous_read(car &myCarData,const SbgLogImuData &myImuData)
{			
	gyroscopes_last=myImuData.gyroscopes[2];

	myCarData.DataReceive();
	carAngle[0]=myCarData.sv1_ang_data;//front wheel angle
	carAngle[1]=myCarData.sv2_ang_data;
	carAngle[2]=myCarData.sv3_ang_data;//back wheel angle,which don't use in this example
	carAngle[3]=myCarData.sv4_ang_data;

	if(nn==0){
		carDistLast[0]=myCarData.dis1_data;//absolute distance of wheel
		carDistLast[1]=myCarData.dis2_data;
		carDistLast[2]=myCarData.dis3_data; 
		carDistLast[3]=myCarData.dis4_data;
		gettimeofday(&timeStampLast, NULL);
		haha.GetWheelCount(carDistLast);//copy carDistLast to Odometry::WheelCount	
	}else{
		carDist[0]=myCarData.dis1_data;//absolute distance of wheel
		carDist[1]=myCarData.dis2_data;
		carDist[2]=myCarData.dis3_data; 
		carDist[3]=myCarData.dis4_data;
		isInitOk = true;
	}								    	    
    nn++;	
}

void DataProcess::else_read(car &myCarData,const SbgLogImuData &myImuData)
{
	gyroscopes =myImuData.gyroscopes[2];//yaw angular velocity
	myCarData.DataReceive();

	//eliminate data's shift by returning to previous state
	if(fabs(myCarData.dis1_data - carDist[0]) < 100) 
		carDist[0]=myCarData.dis1_data;
    if(fabs(myCarData.dis2_data - carDist[1]) < 100) 
    	carDist[1]=myCarData.dis2_data;
    if(fabs(myCarData.dis3_data - carDist[2]) < 100)
    	carDist[2]=myCarData.dis3_data;
    if(fabs(myCarData.dis4_data - carDist[3]) < 100)
    	carDist[2]=myCarData.dis4_data;

	carAngle[0]=myCarData.sv1_ang_data;//front wheel angle
	carAngle[1]=myCarData.sv2_ang_data;
	carAngle[2]=myCarData.sv3_ang_data;//back wheel angle,which don't use in this example
	carAngle[3]=myCarData.sv4_ang_data;

	gettimeofday(&timeStampNow, NULL);//read the unix time-stamp
	delta_time= 1000000L * (timeStampNow.tv_sec - timeStampLast.tv_sec ) + (timeStampNow.tv_usec - timeStampLast.tv_usec);//units,s
	timeStampLast = timeStampNow;
}

 
void DataProcess::theaGyro_calc(void)
{
	if(calc_counts < 200)
	{
	  gyro_shift += gyroscopes;
	  calc_counts++;
	}
	else if(calc_counts == 200)
	{
	  gyro_shift /= 200;
	  calc_counts++;
	}
	else
	{
	  gyroscopes -= gyro_shift;//substract the null shift
	  delta_angle = (gyroscopes_last + gyroscopes) * delta_time * 0.5;//Runge-Kutta integral method
	  gyroscopes_last = gyroscopes;
	  thea_gyro -= delta_angle;//why not add?
	}
}

Odometry& DataProcess::update_odom(car &CarData,const SbgLogImuData &ImuData,int state,int mode)//state-Circle=0,Normal=1,Stright=2  	 
{	//Initialization need twic data
	if(!isInitOk){
		start_calc = true;						
		previous_read(CarData,ImuData);					
	}
	if(start_calc == true){
		else_read(CarData,ImuData);
		theaGyro_calc();			
	    haha.GetOdometry(carDist,carAngle,state,delta_angle);//get Odometry::WheelDistance and Odometry::WheelDir simultaneously
	    li = haha.Update();//根据状态位选择模式,更新相对坐标系下delta x,delta y,delta thea

	    //should it attribute to Update function?
	    //阈值还需要调试
	    if(fabs(li.x)>0.6||fabs(li.y)>0.6||fabs(li.thea)>0.3){
	        cout << " no change!" << endl;
	        li=temp_li;//如果有跳变，则采用上次的变化
	    }

	    //cumulate the delta x,delta y delta thea transfrom to the world frame
	    switch(mode){
	    	case 0:
				trace.x-=cos(trace.thea)*li.x-sin(trace.thea)*li.y;
				trace.y+=cos(trace.thea)*li.y+sin(trace.thea)*li.x;
				trace.thea+=li.thea;//why not add thea priorily then upadte trace.x,trace.y
				trace.thea=0.02*trace.thea+0.98*thea_gyro;//complementary filter
				//limit angle's scale
				if(trace.thea< -PI) trace.thea+=2*PI;
				else if(trace.thea> PI) trace.thea-=2*PI;
	    		break;
	    	case 1:
				trace.thea = thea_gyro;
				// trace.thea = 0.02 * trace.thea + 0.98 * thea_gyro;
				//limit angle's scale
				if(trace.thea<-PI) trace.thea+=2*PI;
				else if(trace.thea>PI) trace.thea-=2*PI;
				cout<<ubuntu_time_show()<<" " << delta_time << " " << trace.thea*57.3 << " " << thea_gyro*57.3 << endl;
	    		break;
	    }

	    //new_heading = true;//2017-12-04,have been deleted

	    //update Odometry::WheelCount
	    if((!(li.x==0&&li.y==0&&li.thea==0)) || mode == 1){
	        memcpy(carDistLast,carDist,4*sizeof(long int));
	          //cout << carDistLast[0] << " " << carDistLast[1] << " " << carDistLast[2] << " " << carDistLast[3] << endl;
	        haha.GetWheelCount(carDistLast);
	    }
	    temp_li=li;
	}
	return haha;
}