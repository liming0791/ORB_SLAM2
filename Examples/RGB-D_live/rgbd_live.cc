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

#include <sys/syscall.h>  
#define gettid() syscall(__NR_gettid)

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>

#include<opencv2/core/core.hpp>

#include<Oni_IMU.h>

#include<System.h>


using namespace std;

char port[20] = "/dev/ttyACM0";

bool ifTrack = false, stopRGBD = false, stopIMU = false;
double avetime = 0;
float IMUData[9];
Oni_IMU* oniDevicePtr = NULL;
cv::Mat *dMatPtr = NULL, *cMatPtr= NULL;
ORB_SLAM2::System* SLAMPtr = NULL;


void trackRGBD()
{

    std::cout << "TrackRGBD pid: " << gettid() << std::endl;

    while(true)
    {
        // Get live kinect data
        long long depthTime, colorTime;
        oniDevicePtr->getFrameData(
                (int16_t* )dMatPtr->data, (unsigned char*)cMatPtr->data, depthTime, colorTime);
        double tframe = depthTime/1000000.0;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAMPtr->TrackRGBD(*cMatPtr,*dMatPtr,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        //printf("tracking time: %f\n", ttrack);

        avetime+=ttrack;
        avetime/=2;

        // Wait to load the next frame, 30Hz
        if(ttrack < 0.033333)
            usleep((0.033333-ttrack)*1e6);
        if(stopRGBD)
            break;
    }    
}

void trackIMU()
{

    std::cout << "TrackIMU pid: " << gettid() << std::endl;

    while(true){

        long long IMUTime = 0;
        oniDevicePtr->getIMUData(IMUData, IMUTime);


        printf("IMU : %f %f %f %f %f %f %f %f %f %d\n", 
            IMUData[0], IMUData[1], IMUData[2], 
            IMUData[3], IMUData[4], IMUData[5], 
            IMUData[6], IMUData[7], IMUData[8], IMUTime);
        SLAMPtr->TrackIMU(IMUData, IMUTime);

        if(stopIMU)
            break;
        usleep(10000);
    }
}

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings reusemap usergbd useimu" << endl;
        return 1;
    }
    
    bool usergbd = atoi(argv[4]);
    bool useimu = atoi(argv[5]);

    //frame params 
    int width = 640, height = 480;
    long long depthTime, colorTime;
    //cv::namedWindow("color");
    //cv::namedWindow("depth");

    //open kinect
    Oni_IMU oniDevice;
    if(useimu)
        oniDevice.initIMU(port, 115200);
    if(usergbd)
        oniDevice.initOni(&width, &height);
    oniDevice.begin();
    oniDevicePtr = &oniDevice;

    //frame data
    cv::Mat imRGB(height, width, CV_8UC3); 
    cv::Mat imD(height, width, CV_16UC1);
    dMatPtr = &imD;
    cMatPtr = &imRGB;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    bool reusemap = atoi(argv[3]);
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true, reusemap);
    SLAMPtr = &SLAM;

    //start trackRGB thread
    std::thread *mpTrackRGBDThread;
    if(usergbd){
        printf("start track rgbd thread\n");
        mpTrackRGBDThread = new std::thread(trackRGBD);
    }
    //start trackIMU threads
    std::thread *mpTrackIMUThread;
    if(useimu){
        printf("start track imu thread\n");
        mpTrackIMUThread = new  std::thread(trackIMU);
    }
        // Main loop
    float totaltime = 0;
    char command = 'a';
    while(true)
    {
        command = getchar();
        if(command=='q')
            break;
    }

    //stop tracking threads
    stopRGBD = stopIMU = true;
    oniDevice.close();

    // Stop all threads
    SLAM.Shutdown();

    // Save Map
    SLAM.SaveMap("Slam_Map.bin");

    // Tracking time statistics
    cout << "-------" << endl << endl;
    cout << "mean tracking time: " << avetime << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    return 0;
}
