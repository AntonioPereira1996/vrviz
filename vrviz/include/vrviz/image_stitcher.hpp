#ifndef _ERGOAWARE_HUMAN_GAZEBO_INCLUDE_ROS_IMAGESTITCHER_HPP_
#define _ERGOAWARE_HUMAN_GAZEBO_INCLUDE_ROS_IMAGESTITCHER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>


namespace ros {

class ImageStitcher {
 public:
    //--------------------------------------------------------------------------
    /// @brief      Synchronizion policy (2 input frames)
    ///
    typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo > SyncPolicy;

    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @param      handle              The handle
    /// @param[in]  left_camera_topic   The left camera topic
    /// @param[in]  right_camera_topic  The right camera topic
    /// @param[in]  topic               The topic
    ///
    ImageStitcher(ros::NodeHandle* handle,
                  const std::string& left_camera_topic,
                  const std::string& right_camera_topic,
                  const std::string& topic = "/rgb/image_raw");

    //--------------------------------------------------------------------------
    /// @brief      Destroys the object.
    ///
    ~ImageStitcher();

 protected:
    //--------------------------------------------------------------------------
    /// @brief      Camera topics callback (syncrhonized)
    ///
    /// @param[in]  image_left   Left camera image
    /// @param[in]  image_right  Right camera image
    ///
    void callback(const sensor_msgs::Image::ConstPtr& image_left, const sensor_msgs::Image::ConstPtr& image_right, const sensor_msgs::CameraInfo::ConstPtr& camera_info);
    void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& camera_info_message);

    //--------------------------------------------------------------------------
    /// @brief     Node handle (for publishing and subscribing)
    ///
    ros::NodeHandle* _handle;

    //--------------------------------------------------------------------------
    /// @brief     Image transport (for publishing and subscribing)
    ///
    image_transport::ImageTransport _transport;

    //--------------------------------------------------------------------------
    /// @brief     Camera subscribers (for synchronizer)
    ///
    image_transport::SubscriberFilter _left_camera_subscriber;
    image_transport::SubscriberFilter _right_camera_subscriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo> _camera_info_subscriber;

    //--------------------------------------------------------------------------
    /// @brief     Syncrhonizer instance
    ///
    message_filters::Synchronizer< SyncPolicy > _synchronizer;

    //--------------------------------------------------------------------------
    /// @brief     Image publisher (stitched/processed image)
    ///
    image_transport::Publisher _publisher;
    ros::Publisher _camera_info_publisher;
};

}  // namespace ros

#endif  // _ERGOAWARE_HUMAN_GAZEBO_INCLUDE_ROS_IMAGESTITCHER_HPP_
