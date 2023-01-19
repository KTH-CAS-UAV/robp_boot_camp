// ROS
#include <angles/angles.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

// STL
#include <cmath>
#include <random>

class World {
 public:
  World(ros::NodeHandle &nh) : tf_listener_(tf_buffer_) {
    vis_pub_ = nh.advertise<visualization_msgs::Marker>("wall_marker", 0, true);
    reset_pub_ = nh.advertise<std_msgs::Empty>("reset_distance", 0);
    reset_srv_ = nh.advertiseService("reset_world", &World::reset, this);
  }

  void publish(tf2::Transform t = tf2::Transform()) {
    // Create random number generators
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(angles::from_degrees(-20.0),
                                         angles::from_degrees(20.0));

    double angle_z = dis(gen);

    visualization_msgs::Marker wall_marker;
    wall_marker.header.frame_id = "odom";
    wall_marker.header.stamp = ros::Time();
    wall_marker.ns = "world";
    wall_marker.id = 0;
    wall_marker.type = visualization_msgs::Marker::CUBE;
    wall_marker.action = visualization_msgs::Marker::ADD;
    wall_marker.scale.x = 500.0;
    wall_marker.scale.y = 0.01;
    wall_marker.scale.z = 0.2;
    wall_marker.color.a = 1.0;
    wall_marker.color.r = (255.0 / 255.0);
    wall_marker.color.g = (0.0 / 255.0);
    wall_marker.color.b = (0.0 / 255.0);

    tf2::Quaternion q = t.getRotation();
    tf2::Vector3 p = t.inverse().getOrigin();

    ROS_INFO("Robot angle: %f", q.getAngle());
    ROS_INFO("Robot axis: %f %f %f", q.getAxis().getX(), q.getAxis().getY(),
             q.getAxis().getZ());
    ROS_INFO("Robot position: %f %f %f", t.getOrigin().getX(),
             t.getOrigin().getY(), t.getOrigin().getZ());

    if (angles::from_degrees(5.0) >= angle_z &&
        angles::from_degrees(0.0) <= angle_z) {
      angle_z += angles::from_degrees(5.0);
    }

    if (angles::from_degrees(-5.0) <= angle_z &&
        angles::from_degrees(0.0) >= angle_z) {
      angle_z -= angles::from_degrees(5.0);
    }

    angle_z += q.getAngle() * (-q.getAxis().getZ());

    p.setX(p.getX() - 0.4 * std::sin(angle_z));
    p.setY(p.getY() + 0.4 * std::cos(angle_z));
    p.setZ(std::isnan(p.getZ()) ? 0.1 : p.getZ() + 0.1);
    q.setRotation(tf2::Vector3(0, 0, 1), angle_z);

    tf2::toMsg(p, wall_marker.pose.position);
    wall_marker.pose.orientation = tf2::toMsg(q);

    vis_pub_.publish(wall_marker);
  }

 private:
  bool reset(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    reset_pub_.publish(std_msgs::Empty());

    tf2::Transform transform;
    bool got_transform = false;
    while (!got_transform) {
      try {
        tf2::fromMsg(
            tf_buffer_.lookupTransform("base_link", "odom", ros::Time(0))
                .transform,
            transform);
        got_transform = true;
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }

    publish(transform);

    return true;
  }

 private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher vis_pub_;
  ros::Publisher reset_pub_;

  ros::ServiceServer reset_srv_;
};

int main(int argc, char **argv) {
  // Set up ROS
  ros::init(argc, argv, "world_node");
  ros::NodeHandle nh;

  World world(nh);
  world.publish();

  ros::spin();

  return 0;
}