#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include <vector>
#include <array>

float Arr[16];
void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Mono_sub");

    ros::NodeHandle n;  

    ros::Subscriber sub3 = n.subscribe("matrix_tl", 100, arrayCallback);

    ros::spin();

    return 0;
}

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            printf("%f,", array->data[i*4 + j]);
        }
        printf("\n");
   }
}
