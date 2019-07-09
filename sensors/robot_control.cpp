#include "robot_control.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include "SimpleSerial.h"
#include "TimeoutSerial.h"
#include <thread>

// include OpenCV header file
#include <opencv2/opencv.hpp>

// dead reckoning
#include "../dead_reckoning/deadreckon.h"
#define DR 1

using namespace std;
using namespace boost;

bool g_robot_thread_run = false;
extern string g_folder_name;
extern bool g_record;

extern int g_key_in;
extern float g_angle;
float dr_x;
float dr_y;

int testSerial()
{
    try {

        SimpleSerial serial("/dev/ttyUSB0",115200);

        serial.writeString("Hello world\n");

        //cout<<"Received : "<<serial.readLine()<<" : end"<<endl;

    } catch(boost::system::system_error& e)
    {
        cout<<"Error: "<<e.what()<<endl;
        return 1;
    }
}

int testTimoutSerial()
{
    try {
 
        TimeoutSerial serial("/dev/ttyUSB0",115200);
        serial.setTimeout(posix_time::seconds(5));

        //Text test
        serial.writeString("Hello world\n");
        //cout<<serial.readStringUntil("\r\n")<<endl;
    
        //Binary test
        char values[]={(char)0xde,(char)0xad,(char)0xbe,(char)0xef};
        serial.write(values,sizeof(values));
        //serial.read(values,sizeof(values));
        for(unsigned int i=0;i<sizeof(values);i++)
        {
            cout<<static_cast<int>(values[i])<<endl;
        }

        serial.close();
  
    } catch(boost::system::system_error& e)
    {
        cout<<"Error: "<<e.what()<<endl;
        return 1;
    }
}

//SimpleSerial g_serial("/dev/ttyUSB0",115200);
TimeoutSerial g_serial("/dev/ttyUSB0",115200);

void sendCommand(char* buf) {

    string strData((char*)buf);
	g_serial.writeString(strData);
    //g_serial.write(buf1,sizeof(buf1));
	char data[4];
	g_serial.read(data, sizeof(data));

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

bool parsingEncoderData(char* buf, int &left, int &right) {
	// send 02 41 43 30 03 31
    // receive 
    // 02 30 --> STX
    // 20 20 31 31 34 38 34 36 38 34 -->left
    // 20 20 31 31 30 36 31 39 30 38 -->right
    // 03 37 --> ETX

	//if (buf[0]==0x02 && buf[1]==0x30) 
	{
		string strtmp1(buf);
		//cout<<"string= "<<strtmp1<<endl;
		try {
			left = stoi(strtmp1.substr(0,10));
			right= stoi(strtmp1.substr(10,10));
			return true;
		} catch (...) {
			cout<<"[encoder] Exception occurred"<<endl;

		}
	}
	return false;


}

bool getEncoder(int &left, int &right){
	char buf[6];
	
	buf[0] = 0x02;
	buf[1] = 'A';
	buf[2] = 'C';
	buf[3] = '0';
	buf[4] = 0x03;
	buf[5] = 0x31;

    string strData((char*)buf);
	g_serial.writeString(strData);

	// read encoder
	char data[24];

/*
	while (1) {
		g_serial.read(data,1);
		if (data[0]==(char)0x02) {
			g_serial.read(data,1);
			if (data[0]==(char)0x30) 
				break;
		}
	}
*/
	while (1) {
		g_serial.read(data,1);
		if (data[0]==(char)0x02) {  // STX
			g_serial.read(data,1);
			if (data[0]==(char)0x30) {
				g_serial.read(data,2);
				if (data[0]==(char)0x03) // ack from robot
					continue;
				else // encoder data
					break;
			}
		}
	}
	

	g_serial.read(&data[2], sizeof(data)-4);
	
	char buf1[1];
	buf1[0] = 0x06;

    string strData1((char*)buf1);
	g_serial.writeString(strData1);

	return parsingEncoderData(data, left, right);


}

void V_Key()
{
    cout<<"V_key"<<endl;

	//TODO: change to velocity control mode
	//// servo motor off
	//unsigned char buf1[7];
	char buf1[7];

	buf1[0] = 0x02;
	buf1[1] = 'D';
	buf1[2] = 'B';
	buf1[3] = '0';
	buf1[4] = '0';
	buf1[5] = 0x03;
	buf1[6] = 0x05;

	sendCommand(buf1);
	
	
	
	//// converting to the velocity mode
	//unsigned char buf2[7];	
	char buf2[7];	

	buf2[0] = 0x02;
	buf2[1] = 'C';
	buf2[2] = 'Z';
	buf2[3] = '1';
	buf2[4] = '1';
	buf2[5] = 0x03;
	buf2[6] = 0x1A;

	sendCommand(buf2);
	
	
	

	//// servo motor on
	char buf3[7];	

	buf3[0] = 0x02;
	buf3[1] = 'D';
	buf3[2] = 'B';
	buf3[3] = '1';
	buf3[4] = '1';
	buf3[5] = 0x03;
	buf3[6] = 0x05;

	sendCommand(buf3);
	
	
	
}


void P_Key()
{
	//TODO: change to position control mode
	//// servo motor off
	cout<<"P_key"<<endl;

	char buf1[7];

	buf1[0] = 0x02;
	buf1[1] = 'D';
	buf1[2] = 'B';
	buf1[3] = '0';
	buf1[4] = '0';
	buf1[5] = 0x03;
	buf1[6] = 0x05;

	sendCommand(buf1);




	//// converting to the velocity mode
	char buf2[7];	

	buf2[0] = 0x02;
	buf2[1] = 'C';
	buf2[2] = 'Z';
	buf2[3] = '0';
	buf2[4] = '0';
	buf2[5] = 0x03;
	buf2[6] = 0x1A;

	sendCommand(buf2);
	



	//// servo motor on
	char buf3[7];	

	buf3[0] = 0x02;
	buf3[1] = 'D';
	buf3[2] = 'B';
	buf3[3] = '1';
	buf3[4] = '1';
	buf3[5] = 0x03;
	buf3[6] = 0x05;

	sendCommand(buf3);

	
}


void Key_Arrow1() 
{
	//// go forward 500/sec
	cout <<"[robot] Key_Arrow1"<<endl;

	char buf1[14];	

	buf1[0]  = 0x02;
	buf1[1]  = 'B';
	buf1[2]  = 'E';
	buf1[3]  = '0';
	buf1[4]  = '1';
	buf1[5]  = '0';
	buf1[6]  = '0';
	buf1[7]  = ';';
	buf1[8]  = '0';
	buf1[9]  = '1';
	buf1[10] = '0';
	buf1[11] = '0';
	buf1[12] = 0x03;
	buf1[13] = buf1[1]^buf1[2]^buf1[3]^buf1[4]^buf1[5]^buf1[6]^buf1[7]^buf1[8]^buf1[9]^buf1[10]^buf1[11]^buf1[12];

	sendCommand(buf1);

}

void Key_Arrow2() 
{
	cout<<"[robot] Key_Arrow2"<<endl;

	//// backward 100/sec

	char buf1[14];	

	buf1[0]  = 0x02;
	buf1[1]  = 'B';
	buf1[2]  = 'E';
	buf1[3]  = '-';
	buf1[4]  = '1';
	buf1[5]  = '0';
	buf1[6]  = '0';
	buf1[7]  = ';';
	buf1[8]  = '-';
	buf1[9]  = '1';
	buf1[10] = '0';
	buf1[11] = '0';
	buf1[12] = 0x03;
	buf1[13] = buf1[1]^buf1[2]^buf1[3]^buf1[4]^buf1[5]^buf1[6]^buf1[7]^buf1[8]^buf1[9]^buf1[10]^buf1[11]^buf1[12];

	sendCommand(buf1);


}


void Key_Arrow3() 
{
	cout<<"[robot] Key_Arrow3"<<endl;
	//// turn left 100/sec

	int i;
	unsigned char temp = 0;
	char buf1[14];	

	buf1[0]  = 0x02;
	buf1[1]  = 'B';
	buf1[2]  = 'E';
	buf1[3]  = '-';
	buf1[4]  = '0';
	buf1[5]  = '5';
	buf1[6]  = '0';
	buf1[7]  = ';';
	buf1[8]  = '0';
	buf1[9]  = '0';
	buf1[10] = '5';
	buf1[11] = '0';
	buf1[12] = 0x03;
	buf1[13] = buf1[1]^buf1[2]^buf1[3]^buf1[4]^buf1[5]^buf1[6]^buf1[7]^buf1[8]^buf1[9]^buf1[10]^buf1[11]^buf1[12];// 0x12;

	//for(i=1; i<13; i++)
	//{
	//	temp = temp^buf1[i];	
	//}

	//buf1[13] = temp;

	sendCommand(buf1);

	//unsigned char temp;
	//temp = buf1[1]^buf1[2]^buf1[3]^buf1[4]^buf1[5];
	//TRACE("lrc: %x\n", temp);

}

void Key_Arrow4() 
{
	cout<<"[robot] Key_Arrow4"<<endl;
	//// turn right 100/sec

	int i;
	unsigned char temp = 0;
	char buf1[14];	

	buf1[0]  = 0x02;
	buf1[1]  = 'B';
	buf1[2]  = 'E';
	buf1[3]  = '0';
	buf1[4]  = '0';
	buf1[5]  = '5';
	buf1[6]  = '0';
	buf1[7]  = ';';
	buf1[8]  = '-';
	buf1[9]  = '0';
	buf1[10] = '5';
	buf1[11] = '0';
	buf1[12] = 0x03;
	buf1[13] = buf1[1]^buf1[2]^buf1[3]^buf1[4]^buf1[5]^buf1[6]^buf1[7]^buf1[8]^buf1[9]^buf1[10]^buf1[11]^buf1[12];

	sendCommand(buf1);

}

void Key_UpRight() 
{
	cout<<"[robot] Key_UpRight"<<endl;

	int i;
	unsigned char temp = 0;
	char buf1[14];	

	buf1[0]  = 0x02;
	buf1[1]  = 'B';
	buf1[2]  = 'E';
	buf1[3]  = '0';
	buf1[4]  = '1';
	buf1[5]  = '0';
	buf1[6]  = '0';
	buf1[7]  = ';';
	buf1[8]  = '0';
	buf1[9]  = '0';
	buf1[10] = '5';
	buf1[11] = '0';
	buf1[12] = 0x03;
	buf1[13] = buf1[1]^buf1[2]^buf1[3]^buf1[4]^buf1[5]^buf1[6]^buf1[7]^buf1[8]^buf1[9]^buf1[10]^buf1[11]^buf1[12];


	sendCommand(buf1);

}

void Key_UpLeft() 
{
	cout<<"[robot] Key_UpLeft"<<endl;

	int i;
	unsigned char temp = 0;
	char buf1[14];	

	buf1[0]  = 0x02;
	buf1[1]  = 'B';
	buf1[2]  = 'E';
	buf1[3]  = '0';
	buf1[4]  = '0';
	buf1[5]  = '5';
	buf1[6]  = '0';
	buf1[7]  = ';';
	buf1[8]  = '0';
	buf1[9]  = '1';
	buf1[10] = '0';
	buf1[11] = '0';
	buf1[12] = 0x03;
	buf1[13] = buf1[1]^buf1[2]^buf1[3]^buf1[4]^buf1[5]^buf1[6]^buf1[7]^buf1[8]^buf1[9]^buf1[10]^buf1[11]^buf1[12];

	sendCommand(buf1);

}

void Key_Space() 
{
	cout<<"[robot] Stop"<<endl;
	//// Stop


	char buf1[14];	

	buf1[0]  = 0x02;
	buf1[1]  = 'B';
	buf1[2]  = 'E';
	buf1[3]  = '0';
	buf1[4]  = '0';
	buf1[5]  = '0';
	buf1[6]  = '0';
	buf1[7]  = ';';
	buf1[8]  = '0';
	buf1[9]  = '0';
	buf1[10] = '0';
	buf1[11] = '0';
	buf1[12] = 0x03;
	buf1[13] = buf1[1]^buf1[2]^buf1[3]^buf1[4]^buf1[5]^buf1[6]^buf1[7]^buf1[8]^buf1[9]^buf1[10]^buf1[11]^buf1[12];

	sendCommand(buf1);

}

void Key_1() 
{
	cout<<"[robot] speed=100"<<endl;
	//// Stop

	char buf1[14];	

	buf1[0]  = 0x02;
	buf1[1]  = 'B';
	buf1[2]  = 'E';
	buf1[3]  = '0';
	buf1[4]  = '2';
	buf1[5]  = '0';
	buf1[6]  = '0';
	buf1[7]  = ';';
	buf1[8]  = '0';
	buf1[9]  = '2';
	buf1[10] = '0';
	buf1[11] = '0';
	buf1[12] = 0x03;
	buf1[13] = buf1[1]^buf1[2]^buf1[3]^buf1[4]^buf1[5]^buf1[6]^buf1[7]^buf1[8]^buf1[9]^buf1[10]^buf1[11]^buf1[12];

	sendCommand(buf1);


}

void Key_2() 
{
	cout<<"[robot] speed=200"<<endl;
	//// Stop

	char buf1[14];	

	buf1[0]  = 0x02;
	buf1[1]  = 'B';
	buf1[2]  = 'E';
	buf1[3]  = '0';
	buf1[4]  = '3';
	buf1[5]  = '5';
	buf1[6]  = '0';
	buf1[7]  = ';';
	buf1[8]  = '0';
	buf1[9]  = '3';
	buf1[10] = '5';
	buf1[11] = '0';
	buf1[12] = 0x03;
	buf1[13] = buf1[1]^buf1[2]^buf1[3]^buf1[4]^buf1[5]^buf1[6]^buf1[7]^buf1[8]^buf1[9]^buf1[10]^buf1[11]^buf1[12];

	sendCommand(buf1);


}

int robot_comm_thread() {
    cout<<"robot comm thread starts"<<endl;

    try {

        //SimpleSerial serial("/dev/ttyUSB0",115200);

        V_Key();

        //cout<<"Received : "<<serial.readLine()<<" : end"<<endl;
	
		//cout<<"file open"<<endl;
		// write to file
	  	ofstream fp;
		string fname = g_folder_name + "/encoder.txt";

    	fp.open(fname);


		int left, right;
		int t_old, left_old, right_old, angle_old;
		
		bool bFirst = true;

#if DR
		DeadReckon dr;
		dr.init(ENCODER_ONLY); // ENCODER_GYRO // ENCODER_ONLY);

		//DRVisualizer vis;
		//vis.init(1000,1000,50);
#endif


		while(g_robot_thread_run) {
			usleep(100000);	

			switch(g_key_in) {
				case A: //65361: 
                Key_Arrow3();
                //g_key_in = A;
				g_key_in = IDLE;
                break;
            case W: //65362: 
				//cout<<"w key case "<<endl;
                Key_Arrow1();
                //g_key_in = W;
				g_key_in = IDLE;
                break;
            case D: //65363: 
                Key_Arrow4();
                //g_key_in = D;
				g_key_in = IDLE;
                break;
            case S: //65364: 
                Key_Arrow2();
                //g_key_in = S;
				g_key_in = IDLE;
                break;
			case Q: //65364: 
                Key_UpLeft();
                //g_key_in = S;
				g_key_in = IDLE;
                break;
			case E: //65364: 
                Key_UpRight();
                //g_key_in = S;
				g_key_in = IDLE;
                break;
            case V: 
                V_Key();
                //g_key_in = V;
				g_key_in = IDLE;
                break;
            case P: 
                P_Key();
                //g_key_in = P;
				g_key_in = IDLE;
                break;
            case SPACE: 
                Key_Space();
                //g_key_in = SPACE;
				g_key_in = IDLE;
                break;
			case KEY_1:
				Key_1();
				g_key_in = IDLE;
				break;
			
			case KEY_2:
				Key_2();
				g_key_in = IDLE;
				break;
			}
			
			bool bOk = getEncoder(left,right);
			if (bOk==false) {
				cout<<"[encoder] getEncoder fails"<<endl;
				continue;
			}

			//boost::posix_time::ptime timeUTC = boost::posix_time::second_clock::universal_time();
			//time_t now = time(NULL);
 			
			//auto now = std::chrono::high_resolution_clock::now();
			std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
			auto duration = now.time_since_epoch();
			auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);

			//cout<<nanoseconds.count()<<","<<left<<","<<right<<","<<g_angle<<endl;

#if DR
           	float d_t, d_l, d_r, d_g;

			if (bFirst) {
				d_t = d_l = d_r = d_g = 0;
				bFirst = false;
			} else {
				d_t = nanoseconds.count() -  t_old;
				d_l = left - left_old;
				d_r = right - right_old;
				d_g = g_angle - angle_old;
				
			}
			t_old = nanoseconds.count();
			left_old = left;
			right_old = right;
			angle_old = g_angle;
	
			POSE2D pose = dr.update(d_l, d_r, d_g);
			//cout<<"[DR] pose = "<<pose.x<<","<<pose.y<<","<<pose.th<<endl;

			dr_x = pose.x;	// temp!!!
			dr_y = pose.y;	// temp!!!

			//vis.visualize(pose);

			//waitKey(1);
#endif
            // write to file
            if (g_record) 
				fp<<nanoseconds.count()<<","<<left<<","<<right<<","<<g_angle<<endl;

			
		}		
		
		
		fp.close();

		cout<<"robot thread end"<<endl;

    } catch(boost::system::system_error& e)
    {
        cout<<"Error: "<<e.what()<<endl;
        return 1;
    }


}


