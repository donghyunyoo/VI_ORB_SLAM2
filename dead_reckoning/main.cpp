// unit test program for DeadReckon class
#include <iostream>
#include <thread>
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h>
#include <fstream>
#include <boost/algorithm/string.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

#include "deadreckon.h"

using namespace std;
using namespace cv;


#define IMG_WIDTH 640
#define IMG_HEIGHT 360
#define IMG_FPS 15 
#define ACCEL_FPS 250 
#define GYRO_FPS 200 

typedef struct {
    float time;
    float left;
    float right;
} ENCODER_DATA;

typedef struct {
    float time;
    float angle;
    float rate;
} GYRO_DATA;

void readData(vector<ENCODER_DATA>& data_enc, vector<GYRO_DATA>& data_gyro) {
    ifstream fs("../sample_data/DR-data-0612-2/encoder.txt");
    ifstream fs2("../sample_data/DR-data-0612-2/gyro.txt");

    string line;
    int count=0;
    if (fs.is_open()) {
        while ( getline (fs,line) )
        {
            cout << line << '\n';
            
            std::vector<std::string> results;
 
            boost::split(results, line, [](char c){return c == ',';});

            ENCODER_DATA data;
            data.time = stof(results[0]);
            data.left = stof(results[1]);
            data.right = stof(results[2]);

            data_enc.push_back(data);
            count++;
        }
        cout<<"total num of encoder= "<<count<<endl;
        fs.close();
    } else cout << "Unable to open file"; 
    

    count = 0;
    if (fs2.is_open()) {
        while ( getline (fs2,line) )
        {
            cout << line << '\n';

            std::vector<std::string> results;
 
            boost::split(results, line, [](char c){return c == ',';});

            GYRO_DATA data;
            data.time = stof(results[0]);
            data.angle = stof(results[1]);
            data.rate = stof(results[2]);

            data_gyro.push_back(data);
            count++;
        }
        cout<<"total num of gyro= "<<count<<endl;
        fs2.close();
    } else cout << "Unable to open file"; 
 
}

class Visualizer {
    Mat plot;
    POSE2D last_pose;
    float scale;
    int width;
    int height;

public:
    void init(int w, int h, float s) {
        width = w;
        height = h;
        scale = s;

        //plot.create(h, w, CV_8UC3);
        plot = Mat::zeros(h,w, CV_8UC3);

        clear();
    }; 

    void visualize(POSE2D p) {
        Point pt1, pt2;
        pt1.x = last_pose.x*scale + width/2;
        pt1.y = last_pose.y*scale + height/2;

        pt2.x = p.x*scale + width/2;
        pt2.y = p.y*scale + height/2;

        //cout<<pt1<<","<<pt2<<endl;
        cv::line(plot, pt1, pt2, Scalar(0,0,255));
        imshow("result", plot);      

        last_pose = p;
    };

    void clear() {
        plot = Scalar(255, 255, 255);
    };

};


int main()
{
    vector<ENCODER_DATA> data_encoder_raw;
    vector<GYRO_DATA> data_gyro_raw;

    readData(data_encoder_raw, data_gyro_raw);

    cout<<data_encoder_raw.size() <<endl;
    cout<<data_gyro_raw.size() <<endl;

    namedWindow("result", WINDOW_AUTOSIZE );

    int gyro_idx = 0;

    DeadReckon dr;
    dr.init(ENCODER_ONLY); // ENCODER_GYRO // ENCODER_ONLY);

    Visualizer vis;
    vis.init(1000,1000,50);

    for (int i=0; i<data_encoder_raw.size(); i++) {
        float d_l; // encoder left
        float d_r; // encoder right
        float d_g; // gyro
        float d_t; // time

        if (i==0) {
            d_t = d_l = d_r = d_g = 0;
        } else {
            float t = data_encoder_raw[i].time;

            while (t>data_gyro_raw[gyro_idx].time) {
                gyro_idx = gyro_idx + 1;
                if (gyro_idx>data_gyro_raw.size())
                    break;
                
            }

            d_t = data_encoder_raw[i].time - data_encoder_raw[i-1].time;
            d_l = data_encoder_raw[i].left- data_encoder_raw[i-1].left;
            d_r = data_encoder_raw[i].right - data_encoder_raw[i-1].right;
            d_g = data_gyro_raw[gyro_idx].angle- data_gyro_raw[gyro_idx-1].angle;
        }
        POSE2D pose = dr.update(d_l, d_r, d_g);
        //cout<<"pose = "<<pose.x<<","<<pose.y<<","<<pose.th<<endl;

        vis.visualize(pose);

        waitKey(1);
    }

    while(1) {
        int iKey = waitKey(10);

        if (iKey==27)
            break;
  }
    
    return 0;
}