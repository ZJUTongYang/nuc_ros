#include <iostream>
#include <nuc/nuc.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nuc_node");

    nuc::NUC the_nuc;

    ros::spin();
    
    return 0;

}