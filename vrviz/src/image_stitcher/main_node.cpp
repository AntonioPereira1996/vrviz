#include <ros/ros.h>
#include <string>
#include "image_stitcher.hpp"

int main(int argc, char *argv[]) {
    //--------------------------------------------------------------------------
    // start ros node
    ros::init(argc, argv, "image_stitcher");
    ros::NodeHandle handle("~");

    //--------------------------------------------------------------------------
    // parse parameters
    auto left_source  = handle.param< std::string >("image_stitcher/left_source", "/left_eye");
    auto right_source = handle.param< std::string >("image_stitcher/right_source", "/right_eye");
    auto output_topic = handle.param< std::string >("image_stitcher/output_topic", "/image");

    //--------------------------------------------------------------------------
    // instantiate stitchr object
    // subscribes to source images and advertises output topic on constructor
    ros::ImageStitcher stitcher(&handle, left_source, right_source, output_topic);

    //--------------------------------------------------------------------------
    ros::spin();
    // ros::Rate rate(30.0);
    // while (1) {
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    return 0;
}
