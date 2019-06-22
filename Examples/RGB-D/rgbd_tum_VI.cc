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

#define REALTIME

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);


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

int main(int argc, char **argv)
{
    if(argc != 8)
    {
        cerr << endl << "Usage: ./project path_to_ORBVOC.TXT path_to_euroc.yaml path_to_imu/data.csv path_to_cam0/data.csv path_to_cam0/data path_to_cam1/data strName" << endl;
        return 1;
    }

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
