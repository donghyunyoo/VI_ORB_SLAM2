/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<System.h>

#include "src/IMU/imudata.h"
#include "src/IMU/configparam.h"
#include <boost/foreach.hpp>
#include <fstream>
#include <time.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <memory>
#include <functional>
#include <atomic>


// include the librealsense C++ header file
#include <librealsense2/rs.hpp>
#include "example.hpp"

// include OpenCV header file
#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>

// robot control
#include "robot_control.h"

// gyro
#include "gyro.h"

#define REALTIME    // for realtime optimization


using namespace std;
using namespace cv;

////////////////////////////////////
// for realsense
// This example assumes camera with depth and color
// streams, and direction lets you define the target stream
enum class direction
{
    to_depth,
    to_color
};

#define IMG_WIDTH 640
#define IMG_HEIGHT 360
#define IMG_FPS 15 
#define ACCEL_FPS 250 
#define GYRO_FPS 200 

bool g_record = false;
bool g_motion_thread_run = false;
bool g_capture_thread_run = false;
extern bool g_robot_thread_run;
extern bool g_gyro_thread_run;
string g_folder_name;
Mat g_color_image;
Mat g_depth_image;
int g_key_in = IDLE;
vector<pair<double,rs2_vector>> g_accel_data;
vector<pair<double,rs2_vector>> g_gyro_data;


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);



inline void wait_on_enter()
{
    std::string dummy;
    std::cout << "Enter to continue..." << std::endl;
    std::getline(std::cin, dummy);
}


typedef struct ImageList
{
    double timeStamp;
    string imgName;
} ICell;

/**
 * @brief load image list 
 * 
 * @param imagePath: path to a file of image list
 * @param iListData: output list 
 */
void loadImageList(char * imagePath, std::vector<ICell> &iListData)
{
    ifstream inf;
    inf.open(imagePath, ifstream::in);
    const int cnt = 2;          // 你要输出的个数

    string line;
    //int i = 0;
    int j = 0;
    size_t comma = 0;
    size_t comma2 = 0;
    ICell temp;
    getline(inf, line);

#ifdef REALTIME
    int idx = 0;
#endif

    while (!inf.eof())
    {
        getline(inf, line);


#ifdef REALTIME
        idx++;
        // downsampling by 3 for realtime processing
        if (idx%3!=0)
            continue;
#endif

        comma = line.find(',', 0);
        //string temp1 = line.substr(0,comma).substr(0,10);
        //temp.timeStamp = (unsigned long)atoi(line.substr(0,comma).c_str());
        //cout <<line.substr(0,comma).c_str()<<endl;
        stringstream ss;
        ss << line.substr(0, comma).c_str();
        ss >> temp.timeStamp ;
        temp.timeStamp = temp.timeStamp / 1e9;

        while (comma < line.size() && j != cnt - 1)
        {

            comma2 = line.find(',', comma + 1);
            //i = atof(line.substr(comma + 1,comma2-comma-1).c_str());
            temp.imgName = line.substr(comma + 1, comma2 - comma - 1).c_str();
            ++j;
            comma = comma2;
        }
        iListData.push_back(temp);
        j = 0;
    }
    //经过调试发现上面的程序多加了最后一行数据，这里去掉最后一行数据
    iListData.pop_back();
    inf.close();

    //  return 0;
}

/**
 * @brief load imu data from a file
 * 
 * @param imuPath: path to image data file
 * @param vimuData: output imu data
 */
void loadIMUFile(char * imuPath, std::vector<ORB_SLAM2::IMUData> &vimuData)
{
    ifstream inf;
    inf.open(imuPath, ifstream::in);
    const int cnt = 7;          // 你要输出的个数

    string line;
    //int i = 0;
    int j = 0;
    size_t comma = 0;
    size_t comma2 = 0;

    //char imuTime[14] = {0};
    double acc[3] = {0.0};
    double grad[3] = {0.0};
    double imuTimeStamp = 0;

    getline(inf, line);

#ifdef REALTIME
    int idx =0;
#endif

    while (!inf.eof())
    {
        getline(inf, line);

#ifdef REALTIME
        idx++;
        // downsampling by 2 for realtime processing
        if (idx%2!=0)
            continue;
#endif


        comma = line.find(',', 0);
        //string temp = line.substr(0,comma);
        stringstream ss;
        ss << line.substr(0, comma).c_str();
        ss >> imuTimeStamp;

        //imuTimeStamp = (unsigned long)atoi(line.substr(0,comma).c_str());

        //cout<<line.substr(0,comma).c_str()<<' ';
        //memcpy(imuTimeStamp,line.substr(0,comma).c_str(),line.substr(0,comma).length);
        while (comma < line.size() && j != cnt - 1)
        {

            comma2 = line.find(',', comma + 1);
            switch (j)
            {
            case 0:
                grad[0] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                break;
            case 1:
                grad[1] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                break;
            case 2:
                grad[2] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                break;
            case 3:
                acc[0] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                break;
            case 4:
                acc[1] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                break;
            case 5:
                acc[2] = atof(line.substr(comma + 1, comma2 - comma - 1).c_str());
                break;
            }
            //cout<<line.substr(comma + 1,comma2-comma-1).c_str()<<' ';
            ++j;
            comma = comma2;
        }
        ORB_SLAM2::IMUData tempImu(grad[0], grad[1], grad[2], acc[0], acc[1], acc[2], imuTimeStamp / 1e9);
        vimuData.push_back(tempImu);
        j = 0;
    }
    //经过调试发现上面的程序多加了最后一行数据，这里去掉最后一行数据
    vimuData.pop_back();

    inf.close();

    //return 0;
}

void StopThreads()
{
    g_capture_thread_run = false;
    g_motion_thread_run = false;
    g_robot_thread_run = false;
    g_gyro_thread_run = false;
}

void motion_thread() {

    cout<<"motion thread start" <<endl;

    rs2::pipeline pipe;
    
    rs2::config cfg;

    rs2::frameset frames;

    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, ACCEL_FPS);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, GYRO_FPS);

    //pipe.start(cfg);

    //t0 = time.time()
    //counter = 0
    g_motion_thread_run = true;

    ofstream fp;
    string fname = g_folder_name + "/imu.txt";

    fp.open(fname.c_str());
    // Start streaming with the given configuration;
    // Note that since we only allow IMU streams, only single frames are produced
    auto profile = pipe.start(cfg);

    cout<<"motion threat start"<<endl;


 
#if 1    
    double t_accel_last = 0;
    double t_gyro_last = 0;

    while (g_motion_thread_run) // Application still alive?
    {
        rs2::frameset frameset = pipe.wait_for_frames();
    
        // Find and retrieve IMU and/or tracking data
        if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
        {
            rs2_vector accel_sample = accel_frame.get_motion_data();
            //std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
            //...

            // Get the timestamp of the current frame
            double ts = accel_frame.get_timestamp();
            if (ts==t_accel_last) continue;
            t_accel_last = ts;

            // Call function that computes the angle of motion based on the retrieved measures
            //algo.process_gyro(gyro_data, ts);
            char cTime[100];
            sprintf(cTime, "%f", ts);
            //cout<<"time:" << cTime <<endl;
            //cout<<"gyro:"<<accel_sample.x<<","<<accel_sample.y<<","<<accel_sample.z<<endl;

        // write to file
            if (g_record) {
                 fp << "accel," << cTime << ","<<accel_sample.x<<","<<accel_sample.y<<","<<accel_sample.z<<endl;  
            }

            g_accel_data.push_back(make_pair(ts,accel_sample));

        }
    
        if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
        {
            rs2_vector gyro_sample = gyro_frame.get_motion_data();
            //std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
            //...

            // Get the timestamp of the current frame
            double ts = gyro_frame.get_timestamp();
            if (ts==t_gyro_last) continue;
            t_gyro_last = ts;

               
            // Call function that computes the angle of motion based on the retrieved measures
            //algo.process_gyro(gyro_data, ts);
            char cTime[100];
            sprintf(cTime, "%f", ts);
            //cout<<"time:" << cTime <<endl;
            //cout<<"gyro:"<<gyro_sample.x<<","<<gyro_sample.y<<","<<gyro_sample.z<<endl;

        // write to file
            if (g_record) {
                 fp << "gyro," << cTime << ","<<gyro_sample.x<<","<<gyro_sample.y<<","<<gyro_sample.z<<endl;  
            }

            g_gyro_data.push_back(make_pair(ts,gyro_sample));

        }
    
        if (rs2::pose_frame pose_frame = frameset.first_or_default(RS2_STREAM_POSE))
        {
            rs2_pose pose_sample = pose_frame.get_pose_data();
            //std::cout << "Pose:" << pose_sample.translation.x << ", " << pose_sample.translation.y << ", " << pose_sample.translation.z << std::endl;
            //...
     
        }
        usleep(2000);
    }

#else


    while(g_motion_thread_run)
    {
        try {
            //Wait for all configured streams to produce a frame
            frames = pipe.wait_for_frames();
            
            // Cast the frame that arrived to motion frame
            auto motion = frames.as<rs2::motion_frame>();
            cout<<"motion"<<endl;
            // If casting succeeded and the arrived frame is from gyro stream
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                // Get the timestamp of the current frame
                double ts = motion.get_timestamp();
                // Get gyro measures
                rs2_vector gyro_data = motion.get_motion_data();
                // Call function that computes the angle of motion based on the retrieved measures
                //algo.process_gyro(gyro_data, ts);
                char cTime[100];
                sprintf(cTime, "%f", ts);
                //cout<<"time:" << cTime <<endl;
                cout<<"gyro:"<<gyro_data.x<<","<<gyro_data.y<<","<<gyro_data.z<<endl;
                
                // write to file
                if (g_record) {
                    fp << "gyro,"<< cTime << ","<<gyro_data.x<<","<<gyro_data.y<<","<<gyro_data.z<<endl;  
                }
            }
            // If casting succeeded and the arrived frame is from accelerometer stream
            if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                double ts = motion.get_timestamp();
                
                // Get accelerometer measures
                rs2_vector accel_data = motion.get_motion_data();
                // Call function that computes the angle of motion based on the retrieved measures
                //algo.process_accel(accel_data);
                char cTime[100];
                sprintf(cTime, "%f", ts);
                //cout<<"time:" << cTime<<endl;
                cout<<"accel:"<<accel_data.x<<","<<accel_data.y<<","<<accel_data.z<<endl;
                // write to file
                if (g_record) {
                    fp << "accel," << cTime << ","<<accel_data.x<<","<<accel_data.y<<","<<accel_data.z<<endl;  
                }
            }
            
        } catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        usleep(10000);
    }
#endif


 
    fp.close();

    pipe.stop();

    cout << "motion thread end" <<endl;
}

void run_realtime_camera_only(char **argv)
{

    cout<<"capture thread start" <<endl;

/*     rs2::context ctx;
    rs2::device dev = ctx.query_devices().front(); // Reset the first device
    dev.hardware_reset();
    rs2::device_hub hub(ctx);
    dev = hub.wait_for_device();
*/
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_BGR8, IMG_FPS);
    cfg.enable_stream(RS2_STREAM_DEPTH, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_Z16, IMG_FPS);
    
    //Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile profile = pipe.start(cfg);
    cout<<profile<<endl;    
    rs2::colorizer c;                     // Helper to colorize depth images
    texture depth_image, color_image;     // Helpers for renderig images

    cout<<"start"<<endl;

    // Define two align objects. One will be used to align
    // to depth viewport and the other to color.
    // Creating align object is an expensive operation
    // that should not be performed in the main loop
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    float       alpha = 0.5f;               // Transparancy coefficient
    direction   dir = direction::to_color;  // Alignment direction


    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;

    cout<<"capture starts"<<endl;
    
    bool bRun = true;
    g_capture_thread_run = true;
	
    
    sleep(1);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    //vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;



    while(g_capture_thread_run)
    {
        try
        {

        
            //Wait for all configured streams to produce a frame
            frames = pipe.wait_for_frames();
    
            if (dir == direction::to_depth)
            {
                // Align all frames to depth viewport
                frames = align_to_depth.process(frames);
            }
            else
            {
                // Align all frames to color viewport
                frames = align_to_color.process(frames);
            }

            //Get each frame
            //rs2::frame color_frame = frames.get_color_frame();
            //rs2::frame depth_frame = frames.get_depth_frame();

            // With the aligned frameset we proceed as usual
            auto depth_frame = frames.get_depth_frame();
            auto color_frame = frames.get_color_frame();
            auto colorized_depth = c.colorize(depth_frame);

            double ts = color_frame.get_timestamp();


            // Creating OpenCV Matrix from a color image
            Mat color_image(Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
            Mat depth_image(Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3, (void*)colorized_depth.get_data(), Mat::AUTO_STEP);

            // Create depth image
            Mat depth16(Size(IMG_WIDTH, IMG_HEIGHT), CV_16U, (void*)depth_frame.get_data());
            Mat depth8u = depth16;
            //depth8u.convertTo( depth8u, CV_8UC1, 255.0/1000 );
            depth8u.convertTo( depth8u, CV_8UC1, 255.0/3000 );  //todo: decide the scale factor


            // copy images for visualization
            g_color_image = color_image.clone();
            g_depth_image = depth_image.clone();
        //Mat img = imread("/home/banggar/Pictures/Screenshot from 2019-06-09 00-20-06.png");

            // Display in a GUI
            namedWindow("Display Image", WINDOW_AUTOSIZE );
            imshow("Display Image", color_image);
            //namedWindow("Display Depth", WINDOW_AUTOSIZE );
            //imshow("Display Depth", depth_image);

            if (g_record) {
                
                char fname[100];

                sprintf(fname, "%s/rgb/%f.png", g_folder_name.c_str(), ts);
                imwrite(fname, color_image);
                sprintf(fname, "%s/depth/%f.png", g_folder_name.c_str(), ts);
                //imwrite(fname, depth8u);
                imwrite(fname, depth16);
                //cout<<"savee images"<<endl;
            }

            usleep(10000);
        
            // processing
            // Main loop
            cv::Mat imRGB, imD;
            
            // Read image and depthmap from camera
            imRGB = g_color_image; 
            imD = g_depth_image;

            if(imRGB.empty())
            {
                cerr << endl << "Failed to capture image from camera"<<endl;
                return;
            }

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

            // Pass the image to the SLAM system
            SLAM.TrackRGBD(imRGB,imD,ts);


#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            
            vTimesTrack.push_back(ttrack);

            int iKey = waitKey(10);
            if (iKey==27)
                StopThreads();
                
            
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

    }

    wait_on_enter();

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<vTimesTrack.size(); ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size()/2] << endl;
    cout << "mean tracking time: " << totaltime/vTimesTrack.size()<< endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   


    pipe.stop();
    cout << "capture thread end"<<endl;




}

void GetIMUData(vector<pair<double,rs2_vector>> accel_data, vector<pair<double,rs2_vector>> gyro_data, double ts, std::vector<ORB_SLAM2::IMUData>& vimuData, int step=2)
{
    int gyro_idx = 0;
    double t_accel_last = 0;
    int nAccel = accel_data.size();

    for (int i=0; i<nAccel; i+=step) 
    {
        double t_accel = accel_data[i].first;
        rs2_vector accel = accel_data[i].second;

        if (t_accel == t_accel_last) continue;  // skip the same data

        if (t_accel > ts) break;    

        // interpolate gyro data
        double t_gyro = gyro_data[gyro_idx].first;

        while (t_gyro<t_accel && gyro_idx<gyro_data.size()) 
        {
            gyro_idx++;
            t_gyro = gyro_data[gyro_idx].first;
        }
        
        double t_gyro_1 = gyro_data[gyro_idx-1].first;
        rs2_vector gyro_1 = gyro_data[gyro_idx-1].second;
        rs2_vector gyro = gyro_data[gyro_idx].second;
        rs2_vector gyro_inter;
        gyro_inter.x = gyro_1.x + (gyro.x-gyro_1.x) / (t_gyro-t_gyro-1) * (t_accel-t_gyro-1);
        gyro_inter.y = gyro_1.y + (gyro.y-gyro_1.y) / (t_gyro-t_gyro-1) * (t_accel-t_gyro-1);
        gyro_inter.z = gyro_1.z + (gyro.z-gyro_1.z) / (t_gyro-t_gyro-1) * (t_accel-t_gyro-1);


        // update last accel data
        t_accel_last = t_accel;

        ORB_SLAM2::IMUData imudata(gyro_inter.x, gyro_inter.y, gyro_inter.z,
                                        accel.x, accel.y, accel.z, t_accel); 
                vimuData.push_back(imudata);


    }

    // remove data upto the last index
    accel_data.erase(accel_data.begin(), accel_data.begin()+nAccel);
    gyro_data.erase(gyro_data.begin(), gyro_data.begin()+gyro_idx);
    
}

int ProcessKeyIn() 
{
    int iKey = waitKeyEx(50);     // waitKey() for predator, waitKeyEx() for lenovo
    if (iKey!=-1) cout <<"[main] key in: "<< iKey<<endl;
    if (iKey==27) 
        return iKey;
    if (iKey=='r') {
        g_record = !g_record;
        if (g_record)
            cout << "recording starts" <<endl;
        else
            cout << "recording ends" <<endl;

    }
    switch(iKey) {
        case 'a': //65361: 
            //Key_Arrow3();
            g_key_in = A;
            break;
        case 'w': //65362: 
            //Key_Arrow1();
            //cout<<"w key"<<endl;
            g_key_in = W;
            break;
        case 'd': //65363: 
            //Key_Arrow4();
            g_key_in = D;
            break;
        case 's': //65364: 
            //Key_Arrow2();
            g_key_in = S;
            break;
        case 'q': //65364: 
            //Key_Arrow2();
            g_key_in = Q;
            break;
        case 'e': //65364: 
            //Key_Arrow2();
            g_key_in = E;
            break;
                        
        case 'v': 
            //V_Key();
            //cout<<"v key"<<endl;
            g_key_in = V;
            break;
        case 'p': 
            //P_Key();
            
            g_key_in = P;
            break;
        case 32: 
            //Key_Space();
            g_key_in = SPACE;
            break;
        case '1':
            g_key_in = KEY_1;
            break;
        case '2':
            g_key_in = KEY_2;
            break;
    }
    return iKey;
}

int run_realtime_vio(char **argv) 
{

    thread t[4];
    if (stoi(argv[3])>1)
        // motion thread
        t[0] = thread(motion_thread);

    if (stoi(argv[3])>2) {
        // robot thread
        g_robot_thread_run = true;
        t[1] = thread(robot_comm_thread);

        // gyro thread
        g_gyro_thread_run = true;
        t[2] = thread(gyro_comm_thread); 
    }

    cout<<"capture thread start" <<endl;

/*     rs2::context ctx;
    rs2::device dev = ctx.query_devices().front(); // Reset the first device
    dev.hardware_reset();
    rs2::device_hub hub(ctx);
    dev = hub.wait_for_device();
*/
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_BGR8, IMG_FPS);
    cfg.enable_stream(RS2_STREAM_DEPTH, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_Z16, IMG_FPS);
    
    //Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile profile = pipe.start(cfg);
    cout<<profile<<endl;    
    rs2::colorizer c;                     // Helper to colorize depth images
    texture depth_image, color_image;     // Helpers for renderig images

    cout<<"start"<<endl;

    // Define two align objects. One will be used to align
    // to depth viewport and the other to color.
    // Creating align object is an expensive operation
    // that should not be performed in the main loop
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    float       alpha = 0.5f;               // Transparancy coefficient
    direction   dir = direction::to_color;  // Alignment direction


    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;

    cout<<"capture starts"<<endl;
    
    bool bRun = true;
    g_capture_thread_run = true;
	
    
    sleep(1);

   // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    ORB_SLAM2::RGBDConfigParam config(argv[2]);

    // image delay to imu 
    double imageMsgDelaySec = config.GetImageDelayToIMU();

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();
    //cout<<"-----------------------------------------------------------------------------"<<endl;
    char *fullPath = new char[500];// = {0};
    char *fullPathD = new char[500];// = {0};
    memset(fullPath, 0, 500);
    memset(fullPathD, 0, 500);

#if 0 
    // read imu data
    //imgData>>imageTimeStamp>>imageName;
    //imuDataFile>>imuTimeStamp>>grad[0]>>grad[1]>>grad[2]>>acc[0]>>acc[1]>>acc[2];
    std::vector<ORB_SLAM2::IMUData> allimuData;
    std::vector<ICell> iListData;
    //loadIMUFile("/home/fyj/Code/C++/LearnVIORB/Examples/ROS/ORB_VIO/v2_03_diff/V2_03_difficult/mav0/imu0/data.csv",allimuData);
    loadIMUFile(argv[3], allimuData);
    //cout<<"loading imu finished"<<endl;
    //loadImageList("/home/fyj/Code/C++/LearnVIORB/Examples/ROS/ORB_VIO/v2_03_diff/V2_03_difficult/mav0/cam0/data.csv",iListData);

    // Here, the number of camera frames of the left camera is taken as the standard, 
    // and the data frame of the left camera image and the image name are stored in iListData.
    loadImageList(argv[4], iListData);
    
    cout<<iListData.size()<<"------------"<<allimuData.size()<<endl;
    cv::waitKey(0);
#endif



/*
    //In order to align the first frame of the imu in the allimuData with the first frame of the image in the iListData, 
    // the imu data before the first frame time of the image in the ListData is removed.
    double ImgFirstTime = iListData[0].timeStamp;
    for (std::size_t j = 0; j < allimuData.size() - 1; j++)
    {
        if (ImgFirstTime - allimuData[j]._t < 1 / 1e4)
        {

            allimuData.erase(allimuData.begin(), allimuData.begin() + j);
            break;
        }
    }

    cout << std::setprecision(13) << "first Img time, first Imu timeStamp: " << iListData[0].timeStamp << ",     " << allimuData[0]._t << endl;
    if (iListData[0].timeStamp - allimuData[0]._t > 1 / 1e4)
        cerr << "the timestamp of first Imu is not equal to the first Img!" << endl;
*/

 
    cv::Mat imRGB, imD, imLeftRect, imRightRect;


    cout << endl << "-------" << endl;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    //vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    
    cout << "Start processing sequence ..." << endl;

    // in order to align imu and camera data, clear the imu data before
    g_accel_data.clear();
    g_gyro_data.clear();

    int idxImu=0;

    while(g_capture_thread_run) 
    {
        try
        {

        
            //Wait for all configured streams to produce a frame
            frames = pipe.wait_for_frames();
    
            if (dir == direction::to_depth)
            {
                // Align all frames to depth viewport
                frames = align_to_depth.process(frames);
            }
            else
            {
                // Align all frames to color viewport
                frames = align_to_color.process(frames);
            }

            //Get each frame
            //rs2::frame color_frame = frames.get_color_frame();
            //rs2::frame depth_frame = frames.get_depth_frame();

            // With the aligned frameset we proceed as usual
            auto depth_frame = frames.get_depth_frame();
            auto color_frame = frames.get_color_frame();
            auto colorized_depth = c.colorize(depth_frame);

            double ts = color_frame.get_timestamp();


            // Creating OpenCV Matrix from a color image
            Mat color_image(Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
            Mat depth_image(Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC3, (void*)colorized_depth.get_data(), Mat::AUTO_STEP);

            // Create depth image
            Mat depth16(Size(IMG_WIDTH, IMG_HEIGHT), CV_16U, (void*)depth_frame.get_data());
            Mat depth8u = depth16;
            //depth8u.convertTo( depth8u, CV_8UC1, 255.0/1000 );
            depth8u.convertTo( depth8u, CV_8UC1, 255.0/3000 );  //todo: decide the scale factor


            // copy images for visualization
            g_color_image = color_image.clone();
            g_depth_image = depth_image.clone();
        //Mat img = imread("/home/banggar/Pictures/Screenshot from 2019-06-09 00-20-06.png");

            // Display in a GUI
            namedWindow("Display Image", WINDOW_AUTOSIZE );
            imshow("Display Image", color_image);
            //namedWindow("Display Depth", WINDOW_AUTOSIZE );
            //imshow("Display Depth", depth_image);

            if (g_record) {
                
                char fname[100];

                sprintf(fname, "%s/rgb/%f.png", g_folder_name.c_str(), ts);
                imwrite(fname, color_image);
                sprintf(fname, "%s/depth/%f.png", g_folder_name.c_str(), ts);
                //imwrite(fname, depth8u);
                imwrite(fname, depth16);
                //cout<<"savee images"<<endl;
            }

            usleep(10000);
        
            // processing
            // Main loop
            cv::Mat imRGB, imD;
            
            // Read image and depthmap from camera
            imRGB = g_color_image; 
            imD = g_depth_image;

            if(imRGB.empty())
            {
                cerr << endl << "Failed to capture image from camera"<<endl;
                return 1;
            }

            // IMU

            std::vector<ORB_SLAM2::IMUData> vimuData;

            GetIMUData(g_accel_data, g_gyro_data, ts, vimuData);
            

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

            cout << "SLAM.TrackRGBDVI" <<endl;
            // Pass the image to the SLAM system
            SLAM.TrackRGBDVI(imRGB, imD, vimuData, ts - imageMsgDelaySec);

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            
            vTimesTrack.push_back(ttrack);

        // comment seems not bad
        bool bstop = false;
        //cout<<"----------------------------------"<<j<<"----------------------------------------"<<endl;
    
        //NOTE  This should be the key to non-real-time. This is just to ensure accuracy, so it doesn't matter if you remove it.
        while (!SLAM.bLocalMapAcceptKF())
        {
            bstop = true;
        };

        bstop = true;

        int iKey = ProcessKeyIn();

        if (iKey==27)
            StopThreads();
        
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

    }

   

#if 0
    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
#endif

    //SLAM.mpViewer->mViewpointX = 0;

    cout << endl << endl << "press any key to shutdown" << endl;
    getchar();
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<vTimesTrack.size(); ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[vTimesTrack.size()/2] << endl;
    cout << "mean tracking time: " << totaltime/vTimesTrack.size()<< endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    delete [] fullPath;
    delete [] fullPathD;
    
    //스레드가 종료될 때 까지 대시
    for (int i = 0; i < 4; ++i) {
        t[i].join();
    }

    return 0;
}

int run_simulation(char **argv)
{

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    ORB_SLAM2::RGBDConfigParam config(argv[2]);

    // image delay to imu 
    double imageMsgDelaySec = config.GetImageDelayToIMU();

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = config.GetAccMultiply9p8();
    //cout<<"-----------------------------------------------------------------------------"<<endl;
    char *fullPath = new char[500];// = {0};
    char *fullPathD = new char[500];// = {0};
    memset(fullPath, 0, 500);
    memset(fullPathD, 0, 500);

    // read imu data
    //imgData>>imageTimeStamp>>imageName;
    //imuDataFile>>imuTimeStamp>>grad[0]>>grad[1]>>grad[2]>>acc[0]>>acc[1]>>acc[2];
    std::vector<ORB_SLAM2::IMUData> allimuData;
    std::vector<ICell> iListData;
    //loadIMUFile("/home/fyj/Code/C++/LearnVIORB/Examples/ROS/ORB_VIO/v2_03_diff/V2_03_difficult/mav0/imu0/data.csv",allimuData);
    loadIMUFile(argv[3], allimuData);
    //cout<<"loading imu finished"<<endl;
    //loadImageList("/home/fyj/Code/C++/LearnVIORB/Examples/ROS/ORB_VIO/v2_03_diff/V2_03_difficult/mav0/cam0/data.csv",iListData);

    // Here, the number of camera frames of the left camera is taken as the standard, 
    // and the data frame of the left camera image and the image name are stored in iListData.
    loadImageList(argv[4], iListData);



    cout<<iListData.size()<<"------------"<<allimuData.size()<<endl;
    cv::waitKey(0);


    //In order to align the first frame of the imu in the allimuData with the first frame of the image in the iListData, 
    // the imu data before the first frame time of the image in the ListData is removed.
    double ImgFirstTime = iListData[0].timeStamp;
    for (std::size_t j = 0; j < allimuData.size() - 1; j++)
    {
        if (ImgFirstTime - allimuData[j]._t < 1 / 1e4)
        {

            allimuData.erase(allimuData.begin(), allimuData.begin() + j);
            break;
        }
    }

    cout << std::setprecision(13) << "first Img time, first Imu timeStamp: " << iListData[0].timeStamp << ",     " << allimuData[0]._t << endl;
    if (iListData[0].timeStamp - allimuData[0]._t > 1 / 1e4)
        cerr << "the timestamp of first Imu is not equal to the first Img!" << endl;


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    int nImages = iListData.size();
    if (nImages < 1)
    {
        cerr << endl << "There is no enough images." << endl;
    }

    vTimesTrack.resize(nImages);



#if 0
    //双目矫正相关参数设置
    if (config._K_l.empty() || config._K_r.empty() || config._P_l.empty() || config._P_r.empty() || config._R_l.empty() || config._R_r.empty() || config._D_l.empty() || config._D_r.empty() ||
            config._rows_l == 0 || config._rows_r == 0 || config._cols_l == 0 || config._cols_r == 0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l, M2l, M1r, M2r;
    cv::initUndistortRectifyMap(config._K_l, config._D_l, config._R_l, config._P_l.rowRange(0, 3).colRange(0, 3), cv::Size(config._cols_l, config._rows_l), CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(config._K_r, config._D_r, config._R_r, config._P_r.rowRange(0, 3).colRange(0, 3), cv::Size(config._cols_r, config._rows_r), CV_32F, M1r, M2r);
#endif

    cv::Mat imRGB, imD, imLeftRect, imRightRect;






    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;



    int idxImu=0;
    for (std::size_t j = 0; j < iListData.size() - 1; j++)
    {
        std::vector<ORB_SLAM2::IMUData> vimuData;
#if 0
        /*
        *imu 的频率是200HZ 图像帧率是20HZ 所以简单的认为每一帧图像对应10个imu数据
        */
        for (unsigned int i = 0; i < 10; i++)
        {
            if (bAccMultiply98)
            {
                allimuData[10 * j + i]._a(0) *= g3dm;
                allimuData[10 * j + i]._a(1) *= g3dm;
                allimuData[10 * j + i]._a(2) *= g3dm;
            }
            ORB_SLAM2::IMUData imudata(allimuData[10 * j + i]._g(0), allimuData[10 * j + i]._g(1), allimuData[10 * j + i]._g(2),
                                       allimuData[10 * j + i]._a(0), allimuData[10 * j + i]._a(1), allimuData[10 * j + i]._a(2), allimuData[10 * j + i]._t); //j*0.0005+i*0.00005
            vimuData.push_back(imudata);
        }
#endif


        while(allimuData[idxImu]._t < iListData[j].timeStamp) {
            // for realsense 
            ORB_SLAM2::IMUData imudata(allimuData[idxImu]._g(0), allimuData[idxImu]._g(1), allimuData[idxImu]._g(2),
                                       allimuData[idxImu]._a(0), allimuData[idxImu]._a(1), allimuData[idxImu]._a(2), allimuData[idxImu]._t); //j*0.0005+i*0.00005
            vimuData.push_back(imudata);
            idxImu++;
        }
        idxImu--;

        if (vimuData.size()==0) {
            cout << "vimuData.size=0" <<endl;
            continue;
        }


        //cout<<"IMU FINISHED READING"<<endl;
        // When the txt is read, an ‘/r’ is added to the image file name, so this character needs to be truncated.
        // The timestamps of the first frame of the imu and img in the dataset are the same, but the SLAM.TrackMonoVI requires the img to be aligned with the last frame of vimudata, so iListData[j+1] is taken here.
        string temp = iListData[j + 1].imgName.substr(0, iListData[j].imgName.size() - 1);
        //sprintf(fullPath,"%s/%s","/home/fyj/Code/C++/LearnVIORB/Examples/ROS/ORB_VIO/v2_03_diff/V2_03_difficult/mav0/cam0/data",temp.c_str());

        //Here, the number of image frames of the left camera is taken as the standard, and in the euroc data set, the names of the images at the same time are the same (named after the time stamp) are all temp
        sprintf(fullPath, "%s/%s", argv[5], temp.c_str());
        sprintf(fullPathD, "%s/%s", argv[6], temp.c_str());

        cout << fullPath<<endl;
        cout <<fullPathD<<endl;

        // Read image and depthmap from file
        imRGB = cv::imread(fullPath, 0);
        imD = cv::imread(fullPathD, 0);
        
        //  cout<<"-----------------------FYJ----------------------"<<iListData[j].timeStamp<<endl;

        //忽略掉最开始的config._testDiscardTime内的时间段
        static double startT = -1;
        if (startT < 0)
            startT = iListData[j + 1].timeStamp;
        if (iListData[j + 1].timeStamp < startT + config._testDiscardTime)
        {
            imRGB = cv::Mat::zeros(imRGB.rows, imRGB.cols, imRGB.type());
            imD = cv::Mat::zeros(imRGB.rows, imRGB.cols, imRGB.type());
        }

        if (imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << fullPath << endl;
            return 1;
        }

        if (imD.empty())
        {
            cerr << endl << "Failed to load depth image at: "
                 << fullPathD << endl;
            return 1;
        }
        memset(fullPath, 0, 500);
        memset(fullPathD, 0, 500);

  
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        cout << "SLAM.TrackRGBDVI" <<endl;
        // Pass the image to the SLAM system
        SLAM.TrackRGBDVI(imRGB, imD, vimuData, iListData[j + 1].timeStamp - imageMsgDelaySec);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[j]=ttrack;

        // comment seems not bad
        bool bstop = false;
        //cout<<"----------------------------------"<<j<<"----------------------------------------"<<endl;
        //NOTE  This should be the key to non-real-time. This is just to ensure accuracy, so it doesn't matter if you remove it.

        
        while (!SLAM.bLocalMapAcceptKF())
        {
            bstop = true;
        };

        bstop = true;
        /*
        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6); */
    }



#if 0
    // Main loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }
#endif

    //SLAM.mpViewer->mViewpointX = 0;

    cout << endl << endl << "press any key to shutdown" << endl;
    getchar();
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    delete [] fullPath;
    delete [] fullPathD;

    return 0;
}

int main(int argc, char **argv)
{
    if (argc==4) // realtime mode
    {
        run_realtime_camera_only(argv);
        //run_realtime_vio(argv);

    } else if(argc != 8)
    {
        cerr << endl << "Usage: ./project path_to_ORBVOC.TXT path_to_euroc.yaml path_to_imu/data.csv path_to_cam0/data.csv path_to_cam0/data path_to_cam1/data strName" << endl;
        cerr << endl << "Usage: ./realsense_rgbd_realtime path_to_vocabulary path_to_settings " << endl;
        return 1;
    } else // simulation mode
    
        run_simulation(argv);


    return 0;

#if 0
    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }
#endif


}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
