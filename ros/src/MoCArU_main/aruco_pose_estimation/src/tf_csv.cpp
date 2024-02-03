#include <fstream>
#include <iostream>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

std::string src_frame, tgt_frame;
std::ofstream out_file;
tf2_ros::Buffer tf_buf;
ros::Time last_stamp;

void onTransformsChanged() {
  // recieve the latest transform
  tf2::Stamped<tf2::Transform> transform;
  try {
    const geometry_msgs::TransformStamped transform_msg =
        tf_buf.lookupTransform(src_frame, tgt_frame, ros::Time());
    tf2::fromMsg(transform_msg, transform);
  } catch (const tf2::TransformException &err) {
    std::cerr << "onTransformsChanged(): " << err.what() << std::endl;
    return;
  }
  if (transform.stamp_ <= last_stamp) {
    // std::cout << "onTransformsChanged(): No update on the transform of interest" << std::endl;
    return;
  }

  //
  const tf2::Vector3 xyz = transform.getOrigin();
  const tf2::Matrix3x3 rmat = transform.getBasis();
  double roll, pitch, yaw;
  rmat.getRPY(roll, pitch, yaw);
  out_file << transform.stamp_.toNSec() << ", " << xyz.getX() << ", " << xyz.getY() << ", "
           << xyz.getZ() << ", " << roll << ", " << pitch << ", " << yaw << std::endl;

  last_stamp = transform.stamp_;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tf_csv");

  // parse args
  if (argc != 4) {
    std::cerr << "Usage: tf_csv source_frame target_frame output_file" << std::endl;
    return 1;
  }

  src_frame = argv[1];
  tgt_frame = argv[2];

  // open or create output csv file
  out_file.open(argv[3]);
  if (!out_file) {
    std::cerr << "Cannot open " << argv[3] << std::endl;
    return 1;
  }
  out_file << "time(nsec), x, y, z, roll(rad), pitch(rad), yaw(rad)" << std::endl;

  ros::NodeHandle nh;
  tf2_ros::TransformListener tf_sub(tf_buf, nh);
  last_stamp = ros::Time(0);
  const boost::signals2::connection tf_conn =
      tf_buf._addTransformsChangedListener(onTransformsChanged);

  ros::spin();

  return 0;
}