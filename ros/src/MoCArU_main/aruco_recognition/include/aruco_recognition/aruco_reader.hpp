#ifndef aruco_RECOGNITION_ARUCO_READER_HPP
#define aruco_RECOGNITION_ARUCO_READER_HPP

#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <image_transport/transport_hints.h>
#include <nodelet/nodelet.h>
#include <object_detection_msgs/Objects.h>
#include <object_detection_msgs/Point.h>
#include <object_detection_msgs/Points.h>
#include <object_detection_msgs/cv_conversions.hpp>
#include <object_detection_msgs/sort.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace aruco_recognition {

class ArUcoReader : public nodelet::Nodelet {
public:
  virtual void onInit() {
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &pnh = getPrivateNodeHandle();

    // load params
    const bool republish_image = pnh.param("republish_image", false);
    aruco_params_ = cv::aruco::DetectorParameters::create();
    pnh.getParam("adaptive_thresh_constant", aruco_params_->adaptiveThreshConstant);
    pnh.getParam("adaptive_thresh_win_size_max", aruco_params_->adaptiveThreshWinSizeMax);
    pnh.getParam("adaptive_thresh_win_size_min", aruco_params_->adaptiveThreshWinSizeMin);
    pnh.getParam("adaptive_thresh_win_size_step", aruco_params_->adaptiveThreshWinSizeStep);
    pnh.getParam("corner_refinement_max_iterations", aruco_params_->cornerRefinementMaxIterations);
#if CV_VERSION_MAJOR >= 3 && CV_VERSION_MINOR >= 3 && CV_VERSION_REVISION <= 0
    // for opencv 3.3.0 or later (i.e. noetic)
    pnh.getParam("corner_refinement_method", aruco_params_->cornerRefinementMethod);
#endif
    pnh.getParam("corner_refinement_min_accuracy", aruco_params_->cornerRefinementMinAccuracy);
    pnh.getParam("corner_refinement_win_size", aruco_params_->cornerRefinementWinSize);
#if CV_VERSION_MAJOR <= 3 && CV_VERSION_MINOR <= 2 && CV_VERSION_REVISION <= 0
    // for opencv 3.2.0 or older (i.e. melodic)
    pnh.getParam("do_corner_refinement", aruco_params_->doCornerRefinement);
#endif
    pnh.getParam("error_correction_rate", aruco_params_->errorCorrectionRate);
    pnh.getParam("marker_border_bits", aruco_params_->markerBorderBits);
    pnh.getParam("max_erroneous_bits_in_border_rate", aruco_params_->maxErroneousBitsInBorderRate);
    pnh.getParam("max_marker_perimeter_rate", aruco_params_->maxMarkerPerimeterRate);
    pnh.getParam("min_corner_distance_rate", aruco_params_->minCornerDistanceRate);
    pnh.getParam("min_distance_to_border", aruco_params_->minDistanceToBorder);
    pnh.getParam("min_marker_distance_rate", aruco_params_->minMarkerDistanceRate);
    pnh.getParam("min_marker_perimeter_rate", aruco_params_->minMarkerPerimeterRate);
    pnh.getParam("min_otsu_std_dev", aruco_params_->minOtsuStdDev);
    pnh.getParam("perspective_remove_ignored_margin_per_cell",
                 aruco_params_->perspectiveRemoveIgnoredMarginPerCell);
    pnh.getParam("perspective_remove_pixel_per_cell", aruco_params_->perspectiveRemovePixelPerCell);
    pnh.getParam("polygonal_approx_accuracy_rate", aruco_params_->polygonalApproxAccuracyRate);
    publish_empty_ = pnh.param("publish_empty", false);
    publish_largest_only_ = pnh.param("publish_largest_only", false);

    // create ArUco's dictionary
    aruco_dict_ = generateArUcoDictionary(nh.param("n_markers", 50), nh.param("marker_size", 4));
    if (!aruco_dict_) {
      throw ros::Exception("ArUcoReader::onInit(): Cannot generate ArUco dictionary");
    }

    // start storing images to be scanned
    image_transport::ImageTransport it(nh);
    static const image_transport::TransportHints default_hints;
    image_sub_ = it.subscribe("image_raw", 1, &ArUcoReader::scanImage, this,
                              image_transport::TransportHints(default_hints.getTransport(),
                                                              default_hints.getRosHints(), pnh));

    // start scanning barcodes
    if (republish_image) {
      image_pub_ = it.advertise("image_out", 1, true);
    }
    aruco_pub_ = nh.advertise<object_detection_msgs::Objects>("aruco_out", 1, true);
  }

private:
  ////////////////////
  // ArUco dictionary
  ////////////////////

  static inline cv::Ptr<cv::aruco::Dictionary> generateArUcoDictionary(const int n_markers,
                                                                       const int marker_size) {
// return a predefined dictionary if exists
#define HCR_RETURN_ARUCO_DICT_IF(M, N)                                                             \
  if (marker_size == M && n_markers == N) {                                                        \
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_##M##X##M##_##N);                    \
  }
    HCR_RETURN_ARUCO_DICT_IF(4, 50);
    HCR_RETURN_ARUCO_DICT_IF(4, 100);
    HCR_RETURN_ARUCO_DICT_IF(4, 250);
    HCR_RETURN_ARUCO_DICT_IF(4, 1000);
    HCR_RETURN_ARUCO_DICT_IF(5, 50);
    HCR_RETURN_ARUCO_DICT_IF(5, 100);
    HCR_RETURN_ARUCO_DICT_IF(5, 250);
    HCR_RETURN_ARUCO_DICT_IF(5, 1000);
    HCR_RETURN_ARUCO_DICT_IF(6, 50);
    HCR_RETURN_ARUCO_DICT_IF(6, 100);
    HCR_RETURN_ARUCO_DICT_IF(6, 250);
    HCR_RETURN_ARUCO_DICT_IF(6, 1000);
    HCR_RETURN_ARUCO_DICT_IF(7, 50);
    HCR_RETURN_ARUCO_DICT_IF(7, 100);
    HCR_RETURN_ARUCO_DICT_IF(7, 250);
    HCR_RETURN_ARUCO_DICT_IF(7, 1000);
#undef HCR_RETURN_ARUCO_DICT_IF
    return cv::Ptr<cv::aruco::Dictionary>();
  }

  /////////////////////////
  // Subscription callback
  /////////////////////////

  void scanImage(const sensor_msgs::ImageConstPtr &input_image) {
    namespace odm = object_detection_msgs;


    // ROS_INFO_STREAM(input_image->header.stamp << " " << ros::Time::now());

    // do nothing if no nodes sbscribe barcode image topic
    if (aruco_pub_.getNumSubscribers() <= 0) {
      return;
    }
    
    // convert input to a mono image
    const cv_bridge::CvImageConstPtr mono_image = cv_bridge::toCvShare(input_image, "mono8");
    if (!mono_image) {
      NODELET_ERROR("ArUcoReader::scanImage(): image conversion error");
      return;
    }

    // scan the mono image
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    cv::aruco::detectMarkers(mono_image->image, aruco_dict_, corners, ids, aruco_params_);
    if (!publish_empty_ && corners.empty()) {
      // no markers found
      return;
    }

    // pack a message of detected markers
    // (use a shared pointer to avoid data copy between nodelets)
    const odm::ObjectsPtr aruco_data(new odm::Objects);
    aruco_data->header = input_image->header;
    for (std::size_t i = 0; i < corners.size(); ++i) {
      // set name
      aruco_data->names.push_back(boost::lexical_cast<std::string>(ids[i]));
      // set location
      odm::Points contour;
      for (const cv::Point2f &corner : corners[i]) {
        odm::Point point;
        point.x = corner.x;
        point.y = corner.y;
        contour.points.push_back(point);
      }
      aruco_data->contours.push_back(contour);
    }

    // erase 2nd largest or less markers if required
    if (publish_largest_only_ && aruco_data->contours.size() >= 2) {
      odm::sortByContour(aruco_data.get(), [](const odm::Points &a, const odm::Points &b) {
        return cv::contourArea(odm::toCvPoints(a)) > cv::contourArea(odm::toCvPoints(b));
      });
      aruco_data->names.resize(1);
      aruco_data->contours.resize(1);
    }

    // publish the result
    if (image_pub_) {
      image_pub_.publish(input_image);
    }
    aruco_pub_.publish(aruco_data);
  }

private:
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher aruco_pub_;

  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
  bool publish_empty_, publish_largest_only_;
};
} // namespace aruco_recognition

#endif
