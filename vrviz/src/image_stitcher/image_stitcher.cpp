#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>     // cv::Mat
#include <cv_bridge/cv_bridge.h>  // cv_bridge::CvImageConstPtr, toCvShare, toImageMsg()
#include "image_stitcher.hpp"

namespace ros {

//------------------------------------------------------------------------------
/// @brief      cv_bridge wrapper, converts ROS images (sensor_msgs::Image) to CV images (cv::Mat)
///
/// @param[in]  ros_image  ROS-type image (sensor_msgs::Image).
///
/// @return     Const pointer (boost::shared_ptr) to cv::Mat instance.
///
inline cv_bridge::CvImageConstPtr toCVImage(const sensor_msgs::Image::ConstPtr& ros_image) {
    cv_bridge::CvImageConstPtr cv_image;
    if (ros_image->encoding == sensor_msgs::image_encodings::RGB8 || ros_image->encoding == sensor_msgs::image_encodings::TYPE_8UC3) {
        // if color (multi-channel) image, invert channels to match OpenCV's default
        cv_image = cv_bridge::toCvShare(ros_image, sensor_msgs::image_encodings::BGR8);
    } else {
        // auto encoding on remaining types
        cv_image = cv_bridge::toCvShare(ros_image);
    }
    return cv_image;
}


//------------------------------------------------------------------------------
/// @brief      cv_bridge wrapper, converts CV images (cv::Mat) to ROS images (sensor_msgs::Image).
///
/// @param[in]  cv_image  CV-type image (cv::Mat)
///
/// @return     Const pointer (boost::shared_ptr) to sensor_msgs::Image instance.
///
inline sensor_msgs::ImagePtr toROSImage(const cv::Mat& cv_image) {
    // parse header, assigining current timestamp
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    
    // parse image enconding
    // @note: OpenCV default is BGR order for color images
    std::string encoding = sensor_msgs::image_encodings::MONO8;
    if (cv_image.type() == CV_8UC3) {
        encoding = sensor_msgs::image_encodings::BGR8;
    } else if (cv_image.type() == CV_16UC3) {
        encoding = sensor_msgs::image_encodings::BGR16;
    } else if (cv_image.type() == CV_16UC1) {
        encoding = sensor_msgs::image_encodings::MONO16;
    } else if (cv_image.type() == CV_32FC1) {
        encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    }
    /* ... */

    // instantiate CvImage on return
    return cv_bridge::CvImage(header, encoding, cv_image).toImageMsg();
}

//------------------------------------------------------------------------------

ImageStitcher::ImageStitcher(ros::NodeHandle* handle, const std::string& left_camera_topic, const std::string& right_camera_topic, const std::string& topic):
    _handle(handle),
    _transport(*handle), _synchronizer(SyncPolicy(10)) {
        // subscribe to source image topics
        _left_camera_subscriber.subscribe(_transport, left_camera_topic + "/rgb/image_raw" , 1);
        _right_camera_subscriber.subscribe(_transport, right_camera_topic + "/rgb/image_raw", 1);
        _camera_info_subscriber.subscribe(*_handle, left_camera_topic + "/rgb/camera_info", 1);
        // setup syncronizer
        _synchronizer.connectInput(_left_camera_subscriber, _right_camera_subscriber, _camera_info_subscriber);
        _synchronizer.registerCallback(boost::bind(&ImageStitcher::callback, this, _1, _2, _3));
        // advertise output topic (stitched image)
        _publisher = _transport.advertise(topic, 1);
        _camera_info_publisher = _handle->advertise<sensor_msgs::CameraInfo>("/camera_info", 1);
}

//------------------------------------------------------------------------------

ImageStitcher::~ImageStitcher() {
    _left_camera_subscriber.unsubscribe();
    _right_camera_subscriber.unsubscribe();
    _camera_info_subscriber.unsubscribe();
}

//------------------------------------------------------------------------------

void ImageStitcher::callback(const sensor_msgs::Image::ConstPtr& image_left, const sensor_msgs::Image::ConstPtr& image_right, const sensor_msgs::CameraInfo::ConstPtr& camera_info) {
    // eval input images
    if (image_left->encoding != sensor_msgs::image_encodings::BGR8 || !image_left->width || !image_left->height) {
        ROS_WARN("-- %s - %s - Invalid/empty input image!", __FILE__, __func__);
        return;
    }
    if (image_right->encoding != sensor_msgs::image_encodings::BGR8 || !image_right->width || !image_right->height) {
        ROS_WARN("-- %s - %s - Invalid/empty input image!", __FILE__, __func__);
        return;
    }

    // cast images to CV type
    cv::Mat cv_image_left, cv_image_right;
    try {
        cv_image_left = toCVImage(image_left)->image;
        cv_image_right = toCVImage(image_right)->image;
    } catch (const cv_bridge::Exception& error) {
        ROS_WARN("-- %s - %s - Unable to conver image to CV type!", __FILE__, __func__);
        return;
    }

    // stitch images together (add right image @ right of left image)
    cv::Mat cv_stitched;
    hconcat(cv_image_left, cv_image_right, cv_stitched);

    // publish resulting image (cast on call)
    auto ros_image = toROSImage(cv_stitched);
    ros_image->header.frame_id = image_left->header.frame_id;
    ros_image->width=image_left->width + image_right->width;
    ros_image->height=image_left->height;
    ros_image->header.stamp=image_left->header.stamp;

    sensor_msgs::CameraInfo camera_info_stitched = *camera_info;
    camera_info_stitched.width = ros_image->width;

    _publisher.publish(ros_image);
    _camera_info_publisher.publish(camera_info_stitched);
}
}  // namespace ros

