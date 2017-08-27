/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza) * For more information see <https://github.com/raulmur/ORB_SLAM2>
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

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include "../../../../include/System.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include <vector>
#include <array>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::Publisher* pub):mpSLAM(pSLAM), mpub(pub){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    ORB_SLAM2::System* mpSLAM;
    ros::Publisher* mpub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_pub");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ros::NodeHandle nodeHandler;
    ros::Publisher pub = nodeHandler.advertise<std_msgs::Float32MultiArray>("matrix_tl", 1000);
    ImageGrabber igb(&SLAM, &pub);

    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv::Mat trans_mat;
    std_msgs::Float32MultiArray array;
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    trans_mat = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    std::cout << trans_mat << std::endl;
    if(trans_mat.rows > 3 && trans_mat.cols > 3){
        array.data.clear();
        for (int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++){
                array.data.push_back(trans_mat.at<float>(i, j));
            }
        }
        mpub->publish(array);
    }
}


