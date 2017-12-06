#ifndef __CONTROL_CAR_H
#define __CONTROL_CAR_H

#include "serial.h"

class car
{
public:	

	car(float speed=0.1);//被赋予的初始值成为默认值。作用是函数调用的时候，如果该参数的默认值正好适用，可以不给该参数传值，比如int test(int i=0); 可以这样调用test()
	void ChangeSpeed(float speed);
	void GetInfo(int mode, int ang=0, float speed=0.5);
	void run();
	void Move(short int speed,int angle);
	void Circle(short int speed);
	void Stop();
	void DataReceive();
	
	float perimeter = 80.110613f;//轮毂周长
        
        short mode_data;
        short angle_data, speed_data;
        //右上为1,逆时针编号1,2,3,4
        int sv1_ac_data, sv2_ac_data, sv3_ac_data, sv4_ac_data;
        float sv1_ang_data, sv2_ang_data, sv3_ang_data, sv4_ang_data;//angle
        
        float cur1_data, cur2_data, cur3_data, cur4_data;
        short m1_data, m2_data, m3_data, m4_data;
        int pul1_data, pul2_data, pul3_data, pul4_data;
        int dis1_data, dis2_data, dis3_data, dis4_data;//distance
        
        short us1_data, us2_data, us3_data, us4_data, us5_data, us6_data, us7_data, us8_data, obstacle_data, dis_keep_data;
        
        float cell1_data, cell2_data, cell3_data, cell4_data, cell5_data, cell6_data, cell7_data, cell8_data, cell9_data, cell10_data, cell11_data, cell12_data;
        float max_voltage_data, min_voltage_data, max_voltage_pos_data, min_voltage_pos_data, voltage_diff_data, avg_voltage_data;
        float total_voltage_data, charge_cur_data, discharge_cur_data, soc_data, temperature1_data, temperature2_data;
        
        float temperature3_data, temperature4_data, temperature5_data, temperature6_data, max_temp_data, min_temp_data;
        float avg_temp_data, envirmnt_temp_data;

        int Rc_data;
        float Wr_data;

private:
	unsigned char data_receive[256];
	unsigned char data_send[8];
	Serial        serial;
    int           fd;
    int			  angle;
    float		  velosity;
    int mode;
};
//指令格式
struct instr
{
	int mode;
	int angle;
	float speed;
};

#endif
