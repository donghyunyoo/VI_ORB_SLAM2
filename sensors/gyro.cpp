// cpp file for microinfinity cruizcore

#include "gyro.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include "SimpleSerial.h"
#include "TimeoutSerial.h"

using namespace std;
//using namespace boost;



//Define constants
//const char COMM_PORT[] = "/dev/ttyUSB2";
const int PACKET_SIZE = 8;
//const int SAMPLES = 1000;


bool g_gyro_thread_run = false;
float g_angle = 0.0;

extern string g_folder_name;
extern bool g_record;

TimeoutSerial g_GyroSerial("/dev/ttyUSB1",115200);

//Define global variables
//int file_descriptor;


int gyro_comm_thread() {
    cout<<"gyro comm thread starts"<<endl;

    try {

        //SimpleSerial serial("/dev/ttyUSB0",115200);

		// reset device
        //g_GyroSerial.writeString("$MIA,I,B,115200,R,10,D,Y,N*89");
        g_GyroSerial.writeString("$MIB,RESET*87");

		//cout<<"file open"<<endl;
		// write to file
	  	ofstream fp;
		string fname = g_folder_name + "/gyro.txt";
    	fp.open(fname.c_str());

		//cout<<"file opened"<<endl;


		float rate, angle;

		while(g_gyro_thread_run) {
			//getEncoder(left,right);

			//boost::posix_time::ptime timeUTC = boost::posix_time::second_clock::universal_time();
			//time_t now = time(NULL);
 			bool success = ccr1050_getvalue(rate, angle);
			if (success==false)
				continue;
				
			//auto now = std::chrono::high_resolution_clock::now();
			std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
			auto duration = now.time_since_epoch();
			auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);

			
			// cout<<nanoseconds.count()<<": "<<angle<<","<<rate<<endl;
            // write to file
           	if (g_record) 
				fp<<nanoseconds.count()<<","<<angle<<","<<rate<<endl;

			g_angle = angle;
			
			usleep(1000);	
		}		
		
		
		fp.close();

		cout<<"gyro thread end"<<endl;

    } catch(boost::system::system_error& e)
    {
        cout<<"Error: "<<e.what()<<endl;
        return 1;
    }


}



/*
// Open serial port
bool ccr1050_init()
{
	if(-1 == (file_descriptor = open(COMM_PORT,O_RDONLY)))
	{
		cout << "Error opening port \n";
		cout << "Set port parameters using the following Linux command:\n stty -F /dev/ttyUSB0 115200 raw\n";
		cout << "You may need to have ROOT access";
		return false;
	}
	cout << "CruizCoreR1050 communication port is ready\n";
	return true;
}

// Close serial port
void ccr1050_close()
{
	close(file_descriptor);
	cout << "Closing communication port";
}
*/
// Get a data packet and parse it
bool ccr1050_getvalue(float &rate, float &angle)
{
	short header;
	short rate_int;
	short angle_int;
	float rate_float;
	float angle_float;
	short check_sum;
	char data_packet[PACKET_SIZE];

	//if(PACKET_SIZE != read(file_descriptor,data_packet,PACKET_SIZE))
	// return false;
	while (1) {
		g_GyroSerial.read(data_packet,1);
		if (data_packet[0]==(char)0xFF) {
			g_GyroSerial.read(data_packet,1);
			if (data_packet[0]==(char)0xFF) 
				break;
		}
	}
	//cout <<"head found"<<endl;
	
	//cout<<str<< endl;
/*
	memcpy(&header,data_packet,sizeof(short));
	if(header != (short)0xFFFF)
	{
		cout << "Header error !!!\n";
		return false;
	}
*/
	g_GyroSerial.read(data_packet,PACKET_SIZE-2);

/*
	//cout<<data_packet<<endl;
	// Verify data packet header 
	memcpy(&header,data_packet,sizeof(short));
	//cout<<header<<endl;
	//cout<<(short)0xFFFF<<endl;
	if(header != (short)0xFFFF)
	{
		cout << "Header error !!!\n";
		return false;
	}
*/
	// Copy values from data string 
	memcpy(&rate_int,data_packet+0,sizeof(short));
	memcpy(&angle_int,data_packet+2,sizeof(short));
	memcpy(&check_sum,data_packet+4,sizeof(short));

	// Verify checksum
	if(check_sum != (short)(0xFFFF + rate_int + angle_int))
	{
		cout<< "[gyro] Checksum error!!\n";
		return false;
	}

	// Apply scale factors
	rate_float = rate_int/100.0;
 	angle_float = angle_int/100.0;
	
	//cout << "rate_float:" << rate_float << " [deg/sec]\t angle_float:" << angle_float << " [deg]\n";

	rate = rate_float;
	angle = angle_float;

	return true;
}

