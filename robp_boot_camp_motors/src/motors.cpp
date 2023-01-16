// ROBP
#include <robp_boot_camp_motors/kobuki_motors.h>
#include <robp_msgs/DutyCycles.h>
#include <robp_msgs/Encoders.h>

// ROS
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// STL
#include <memory>
#include <random>
#include <vector>

class Motors {
 public:
  Motors(ros::NodeHandle &nh) {
    kobuki_motors_ = std::make_unique<KobukiMotors>();

    encoders_pub_ = nh.advertise<robp_msgs::Encoders>("/motor/encoders", 1);
    twist_pub_ =
        nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

    duty_cycles_sub_ = nh.subscribe("/motor/duty_cycles", 1,
                                    &Motors::dutyCyclesCallback, this);
  }

  // [0] corresponds to left wheel, [1] corresponds to right wheel
  void dutyCyclesCallback(robp_msgs::DutyCycles::ConstPtr const &msg) {
    duty_cycles_[0] = msg->duty_cycle_left;
    duty_cycles_[1] = msg->duty_cycle_right;
    duty_cyles_time_ = ros::Time::now();
  }

  void updateMotors() {
    // if more than 0.5 seconds have passed and no messages have been received,
    // shutdown the motors
    if ((ros::Time::now() - duty_cyles_time_).toSec() > 0.5) {
      duty_cycles_[0] = 0.0;
      duty_cycles_[1] = 0.0;
    }

    if (1.0 < std::abs(duty_cycles_[0]) || 1.0 < std::abs(duty_cycles_[1])) {
      ROS_FATAL("Duty cycles should be between [-1, 1]");
      exit(1);
    }

    // [0] corresponds to left wheel, [1] corresponds to right wheel
    std::vector<double> wheel_angular_velocities(2, 0.0);
    std::vector<int> abs_encoders(2, 0);
    std::vector<int> diff_encoders(2, 0);

    std::vector<int> pwm{static_cast<int>(255 * duty_cycles_[0]),
                         static_cast<int>(255 * duty_cycles_[1])};

    kobuki_motors_->update(pwm, wheel_angular_velocities, abs_encoders,
                           diff_encoders);

    // publish encoders
    encoders_msg_.header.stamp = ros::Time::now();
    encoders_msg_.encoder_left = abs_encoders[0];
    encoders_msg_.encoder_right = abs_encoders[1];
    encoders_msg_.delta_encoder_left = diff_encoders[0];
    encoders_msg_.delta_encoder_right = diff_encoders[1];
    encoders_msg_.delta_time_left = 1000.0 * 1.0 / 10.0;
    encoders_msg_.delta_time_right = 1000.0 * 1.0 / 10.0;

    encoders_pub_.publish(encoders_msg_);

    // calculate kinematics and send twist to robot simulation node
    geometry_msgs::Twist twist_msg;

    twist_msg.linear.x =
        (wheel_angular_velocities[1] + wheel_angular_velocities[0]) *
        wheel_radius_ / 2.0;
    twist_msg.angular.z =
        (wheel_angular_velocities[1] - wheel_angular_velocities[0]) *
        wheel_radius_ / base_;

    twist_pub_.publish(twist_msg);
  }

 private:
  std::unique_ptr<KobukiMotors> kobuki_motors_;

  ros::Publisher encoders_pub_;
  ros::Publisher twist_pub_;

  ros::Subscriber duty_cycles_sub_;

  // [0] corresponds to left wheel, [1] corresponds to right wheel
  std::array<double, 2> duty_cycles_{};
  ros::Time duty_cyles_time_{ros::Time::now()};

  robp_msgs::Encoders encoders_msg_;

  double wheel_radius_{0.0352};
  double base_{0.23};
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "motors_node");
  ros::NodeHandle nh;

  Motors motors(nh);

  // Control @ 10 Hz
  ros::Rate r(10.0);
  while (nh.ok()) {
    motors.updateMotors();
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}