#include <sbgEComLib.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <math.h>
#include "control_car.h"
#include "DataProcess.h"

using namespace std;

#define  Ri   6.378140e+6              //地球半径
#define  g    9.8                      //重力加速度
#define  e    1/298.257                //偏心率
#define  PI   3.14159265358979323846   //圆周率
#define  Wie  7.292125241172469e-5     //地球自转角速率
#define  RAD  PI/180                   //角度到弧度
#define  IRAD 180/PI                   //弧度到角度

//----------------------------------------------------------------------//
//  Call backs                                                          //
//----------------------------------------------------------------------//

/*!
 *	Callback definition called each time a new log is received.
 *	\param[in]	pHandle									Valid handle on the sbgECom instance that has called this callback.
 *	\param[in]	msgClass								Class of the message we have received
 *	\param[in]	msg										Message ID of the log received.
 *	\param[in]	pLogData								Contains the received log data as an union.
 *	\param[in]	pUserArg								Optional user supplied argument.
 *	\return												SBG_NO_ERROR if the received log has been used successfully.
 */
double euler_roll=0,euler_pitch=0,euler_yaw=0;

int state_now=1,mode_now=0;
car car_data;
DataProcess dataP(0.0,0.0,0.0);

SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
	//
	// Handle separately each received data according to the log ID
	//

    //主要修改这里
	switch (msg)
	{

		case SBG_ECOM_LOG_EKF_EULER:

			euler_roll=pLogData->ekfEulerData.euler[0]*IRAD;
			euler_pitch=pLogData->ekfEulerData.euler[1]*IRAD;
			euler_yaw=pLogData->ekfEulerData.euler[2]*IRAD;
			/*printf("Euler Angles: %3.1f\t%3.1f\t%3.1f\tStd Dev:%3.1f\t%3.1f\t%3.1f   \r", 
				sbgRadToDegF(pLogData->ekfEulerData.euler[0]), sbgRadToDegF(pLogData->ekfEulerData.euler[1]), sbgRadToDegF(pLogData->ekfEulerData.euler[2]), 
				sbgRadToDegF(pLogData->ekfEulerData.eulerStdDev[0]), sbgRadToDegF(pLogData->ekfEulerData.eulerStdDev[1]), sbgRadToDegF(pLogData->ekfEulerData.eulerStdDev[2]));*/
			break;
			

		case SBG_ECOM_LOG_IMU_DATA:
		
			dataP.update_odom(car_data,pLogData->imuData,state_now,mode_now);
			break;
		
		default:

			break;
	}
	return SBG_NO_ERROR;
}

//----------------------------------------------------------------------//
//  Signal Handler                                                      //
//----------------------------------------------------------------------//

/*!
 *	Set a timed task to print sensor informations.
 *	\param[in]	signo									signal.
 *	\return												none.
 */
double acce_sum[3]={0},acce_avg[3]={0};
double gyro_sum[3]={0},gyro_avg[3]={0};
size_t count=0;
ofstream outfile;
void signalHandler(int signo)  
{  
    switch (signo){  
        case SIGALRM: 
			break;            
   }  
}

//----------------------------------------------------------------------//
//  Main program                                                        //
//----------------------------------------------------------------------//

/*!
 *	Main entry point.
 *	\param[in]	argc		Number of input arguments.
 *	\param[in]	argv		Input arguments as an array of strings.
 *	\return					0 if no error and -1 in case of error.
 */

int main(int argc, char** argv)
{
	
	SbgEComHandle			comHandle;
	SbgErrorCode			errorCode;
	SbgInterface			sbgInterface;
	int32				retValue = 0;
	SbgEComDeviceInfo		deviceInfo;

	// Create an interface: 
	// We can choose either a serial for real time operation, or file for previously logged data parsing
	// Note interface closing is also differentiated !
	//
	errorCode = sbgInterfaceSerialCreate(&sbgInterface, "/dev/ttyUSB0", 115200);		// Example for Unix using a FTDI Usb2Uart converter
	//errorCode = sbgInterfaceSerialCreate(&sbgInterface, "COM3", 115200);							// Example for Windows serial communication

	//
	// Test that the interface has been created
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Create the sbgECom library and associate it with the created interfaces
		//
		errorCode = sbgEComInit(&comHandle, &sbgInterface);

		//
		// Test that the sbgECom has been initialized
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Get device inforamtions
			//
			errorCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo);

			//
			// Display device information if no error
			//
			if (errorCode == SBG_NO_ERROR)
			{
				printf("Device : %0.9u found\n", deviceInfo.serialNumber);
			}
			else
			{
				fprintf(stderr, "ellipseMinimal: Unable to get device information.\n");
			}

			//
			// Configure some output logs to 100 Hz
			//
			
			if (sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_DIV_2) != SBG_NO_ERROR)
			{
				fprintf(stderr, "ellipseMinimal: Unable to configure output log SBG_ECOM_LOG_IMU_DATA.\n");
			}
	        if (sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DIV_2) != SBG_NO_ERROR)
			{
				fprintf(stderr, "ellipseMinimal: Unable to configure output log SBG_ECOM_LOG_EKF_EULER.\n");
			}
			//
			// Display a message for real time data display
			//
			printf("sbgECom properly Initialized.\n");
			printf("sbgECom version %s\n\n", SBG_E_COM_VERSION_STR);
			printf("Euler Angles display with estimated standard deviation.\n");

			//
			// Define callbacks for received data
			//		
			sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, NULL);

			//
			//output sensor's data
			//
			outfile.open("/home/syaru/桌面/零漂.txt");

			//
			//Set a timed task to print sensor informations
			//
			signal(SIGALRM, signalHandler); 

			struct itimerval new_value, old_value;  
		    new_value.it_value.tv_sec = 0;  
		    new_value.it_value.tv_usec = 1;  
		    new_value.it_interval.tv_sec = 0;  
		    new_value.it_interval.tv_usec = 100000;  
		    setitimer(ITIMER_REAL, &new_value, &old_value);

			//
			// Loop until the user exist
			//
			//cout << "inside" << endl;  			
			while (1)
			{
				
				errorCode = sbgEComHandle(&comHandle);
				
				//
				// Test if we have to release some CPU (no frame received)
				//
				if (errorCode == SBG_NOT_READY)
				{
					//
					// Release CPU
					//
					sbgSleep(1);
				}
				else
				{
					fprintf(stderr, "Error\n");
				}
			}

			//
			// Close the sbgEcom library
			//
			sbgEComClose(&comHandle);
		}
		else
		{
			//
			// Unable to initialize the sbgECom
			//
			fprintf(stderr, "ellipseMinimal: Unable to initialize the sbgECom library.\n");
			retValue = -1;
		}
		
		//
		// Close the interface
		//
		sbgInterfaceSerialDestroy(&sbgInterface);		
	}
	else
	{
		//
		// Unable to create the interface
		//
		fprintf(stderr, "ellipseMinimal: Unable to create the interface.\n");
		retValue = -1;
	}

	//
	// Returns -1 if we have an error
	//
	return retValue;
}
