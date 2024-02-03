#ifndef aruco_RELATIVE_POSE_ESTIMATOR_HPP
#define aruco_RELATIVE_POSE_ESTIMATOR_HPP

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nodelet/nodelet.h>
#include <object_detection_msgs/Objects.h>
#include <object_detection_msgs/cv_conversions.hpp>
#include <ros/node_handle.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2/transform_datatypes.h>             // tf2::Transform, Quaternion, Vector3, ...
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // tf2::toMsg()
#include <tf2_ros/transform_broadcaster.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <boost/range/algorithm/find.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace aruco_pose_estimation {

class RelativePoseEstimator : public nodelet::Nodelet {
public:
  virtual void onInit() {
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());

    // load params
    target_name_ = pnh.param<std::string>("target_name", "ID0");
    //remove ID from string
    target_name_.erase(0, 2);
    marker_frame_id_ = pnh.param<std::string>("marker_frame_id", "aruco");
    camera_frame_id_ = pnh.param<std::string>("camera_frame_id", "camera");
    exclusive_transforms_ = transformsParam(pnh, "exclusive_transforms", {});
    marker_length_ = pnh.param("marker_length", 0.225);
    const int queue_size = pnh.param("queue_size", 10);
    

    // detection result subscribers
    camera_sub_.subscribe(nh, "camera_info", 1);
    aruco_sub_.subscribe(nh, "aruco_data", 1);

    // callback on synchronized results
    sync_sub_.reset(new SyncSubscriber(queue_size));
    sync_sub_->connectInput(camera_sub_, aruco_sub_);
    sync_sub_->registerCallback(&RelativePoseEstimator::estimatePose, this);
  }

protected:
  void estimatePose(const sensor_msgs::CameraInfoConstPtr &camera_msg,
                    const object_detection_msgs::ObjectsConstPtr &aruco_msg) {
    // find the target object by name, or just take the first object
    const std::size_t obj_id = boost::range::find(aruco_msg->names, target_name_) - aruco_msg->names.begin();
    if (aruco_msg->names.size() == 0 || obj_id >= aruco_msg->names.size() ) {
      // no object given or found by name
      return;
    }
    // calc transform to the marker by the first ArUco detection result
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
        std::vector<std::vector<cv::Point2f>>(
            1, object_detection_msgs::toCvPoints2f(aruco_msg->contours[obj_id])),
        marker_length_, cv::Matx33d(&camera_msg->K[0]), camera_msg->D, rvecs, tvecs);

    // publish transform from marker to camera, and exclusive transforms
    std::vector<geometry_msgs::TransformStamped> transforms;
    // exclusive transforms
    // (useful to isolate transforms from other markers to camera)
    for (const std::pair<std::string, std::string> &e : exclusive_transforms_) {
      transforms.push_back(toTransformMsg(aruco_msg->header.stamp, e.first, e.second,
                                          tf2::Transform::getIdentity()));
    }
    // transform from marker to camera
    transforms.push_back(toTransformMsg(aruco_msg->header.stamp, marker_frame_id_, camera_frame_id_,
                                        toTransform(rvecs[0], tvecs[0]).inverse()));
    tf_pub_.sendTransform(transforms);
  }

  static tf2::Transform toTransform(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
    cv::Matx33d rmat;
    cv::Rodrigues(rvec, rmat);

    tf2::Quaternion rqtn;
    tf2::Matrix3x3(rmat(0, 0), rmat(0, 1), rmat(0, 2), rmat(1, 0), rmat(1, 1), rmat(1, 2),
                   rmat(2, 0), rmat(2, 1), rmat(2, 2))
        .getRotation(rqtn);

    return tf2::Transform(rqtn, tf2::Vector3(tvec[0], tvec[1], tvec[2]));
  }

  static geometry_msgs::TransformStamped toTransformMsg(const ros::Time &stamp,
                                                        const std::string &frame_id,
                                                        const std::string &child_frame_id,
                                                        const tf2::Transform &transform) {
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame_id;
    msg.transform = tf2::toMsg(transform);
    return msg;
  }

  std::vector<std::pair<std::string, std::string>>
  transformsParam(ros::NodeHandle &nh, const std::string &key,
                  const std::vector<std::pair<std::string, std::string>> &default_val) {
    XmlRpc::XmlRpcValue param;
    if (!nh.getParam(key, param)) {
      return default_val;
    }

    try {
      std::vector<std::pair<std::string, std::string>> val;
      for (int i = 0; i < param.size(); ++i) {
        val.push_back(
            {static_cast<std::string>(param[i][0]), static_cast<std::string>(param[i][1])});
      }
      return val;
    } catch (const XmlRpc::XmlRpcException &error) {
      NODELET_ERROR_STREAM("RelativePoseEstimator::transformsParam(): "
                           << error.getMessage() << " (" << error.getCode() << ")");
      return default_val;
    }
  }

protected:
  using SyncSubscriber =
      message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, object_detection_msgs::Objects>;

  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_sub_;
  message_filters::Subscriber<object_detection_msgs::Objects> aruco_sub_;
  std::unique_ptr<SyncSubscriber> sync_sub_;
  tf2_ros::TransformBroadcaster tf_pub_;

  bool has_target_name_;
  std::string target_name_;
  std::string marker_frame_id_, camera_frame_id_;
  double marker_length_;

  std::vector<std::pair<std::string, std::string>> exclusive_transforms_;
};
} // namespace aruco_pose_estimation

#endif