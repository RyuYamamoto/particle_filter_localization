#include "particle_filter_localization/particle_filter_localization.hpp"

ParticleFilterLocalization::ParticleFilterLocalization() : Node("particle_filter_localization")
{
  particle_filter_ptr_ = std::make_shared<ParticleFilter>(100);

  particle_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("particle_pose", 1);
  particle_array_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("particle_array", 1);
  ndt_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ndt_pose", 5,
    std::bind(&ParticleFilterLocalization::callbackNDTPose, this, std::placeholders::_1));
  twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", 5,
    std::bind(&ParticleFilterLocalization::callbackTwist, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::microseconds(static_cast<int>(100 * 0.01)),
    std::bind(&ParticleFilterLocalization::timerCallback, this));
}

void ParticleFilterLocalization::predict() {}

void ParticleFilterLocalization::measure() {}

void ParticleFilterLocalization::timerCallback()
{
  if (!init_) return;

  pubilshParticle(now());

  particle_pose_publisher_->publish(estimatedPose());
}

void ParticleFilterLocalization::callbackTwist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  if (!init_) return;
  if (previous_twist_ptr_ == nullptr) {
    previous_twist_ptr_ = msg;
    return;
  }
  double dt =
    (rclcpp::Time(msg->header.stamp) - rclcpp::Time(previous_twist_ptr_->header.stamp)).seconds();

  particle_filter_ptr_->predict(msg->twist.twist.linear.x, msg->twist.twist.angular.z, dt);

  previous_twist_ptr_ = msg;
}

void ParticleFilterLocalization::callbackNDTPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_with_covariance_ = *msg;

  Eigen::VectorXd initial_pose(6);
  initial_pose(0) = msg->pose.pose.position.x;
  initial_pose(1) = msg->pose.pose.position.y;
  initial_pose(2) = msg->pose.pose.position.z;

  tf2::Quaternion quat(
    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 mat(quat);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  initial_pose(3) = roll;
  initial_pose(4) = pitch;
  initial_pose(5) = yaw;

  if (!init_) {
    particle_filter_ptr_->init(initial_pose, 100);
    init_ = true;
  }

  particle_filter_ptr_->measure(initial_pose);
  particle_filter_ptr_->resampling();
}

void ParticleFilterLocalization::pubilshParticle(const rclcpp::Time stamp)
{
  std::vector<Particle> particles = particle_filter_ptr_->getParticles();

  geometry_msgs::msg::PoseArray partile_array;
  partile_array.header.frame_id = "map";
  partile_array.header.stamp = stamp;

  for (std::size_t i = 0; i < particles.size(); i++) {
    const auto particle_pose = particles.at(i).pose;

    geometry_msgs::msg::Pose pose;
    pose.position.x = particle_pose(0);
    pose.position.y = particle_pose(1);
    pose.position.z = particle_pose(2);
    tf2::Quaternion quat;
    quat.setRPY(particle_pose(3), particle_pose(4), particle_pose(5));
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    partile_array.poses.emplace_back(pose);
  }
  particle_array_publisher_->publish(partile_array);
}

// 重み付き平均ver.
geometry_msgs::msg::PoseStamped ParticleFilterLocalization::estimatedPose()
{
  geometry_msgs::msg::PoseStamped estimated_pose;
  double weight_sum = 0.0;
  Eigen::VectorXd p_sum(6);
  p_sum = Eigen::VectorXd::Zero(6);

  const auto particles = particle_filter_ptr_->getParticles();
  for(auto particle : particles){
    weight_sum += particle.weight;
    p_sum += (particle.pose * particle.weight);
  }
  Eigen::VectorXd mean_pose = p_sum / weight_sum;

  estimated_pose.header.frame_id = "map";
  estimated_pose.header.stamp = now();
  estimated_pose.pose.position.x = mean_pose(0);
  estimated_pose.pose.position.y = mean_pose(1);
  estimated_pose.pose.position.z = mean_pose(2);
  tf2::Quaternion quat;
  quat.setRPY(mean_pose(3), mean_pose(4), mean_pose(5));
  estimated_pose.pose.orientation.x = quat.x();
  estimated_pose.pose.orientation.y = quat.y();
  estimated_pose.pose.orientation.z = quat.z();
  estimated_pose.pose.orientation.w = quat.w();


  return estimated_pose;
}

// 最大尤度ver.
#if 0
geometry_msgs::msg::PoseStamped ParticleFilterLocalization::estimatedPose()
{
  geometry_msgs::msg::PoseStamped estimated_pose;
  Eigen::VectorXd pose_vector(6);
  double max_weight = 0.0;

  const auto particles = particle_filter_ptr_->getParticles();
  for(auto particle : particles){
    if(max_weight < particle.weight){
      max_weight = particle.weight;
      pose_vector = particle.pose;
    }
  }
  estimated_pose.header.frame_id = "map";
  estimated_pose.header.stamp = now();
  estimated_pose.pose.position.x = pose_vector(0);
  estimated_pose.pose.position.y = pose_vector(1);
  estimated_pose.pose.position.z = pose_vector(2);
  tf2::Quaternion quat;
  quat.setRPY(pose_vector(3), pose_vector(4), pose_vector(5));
  estimated_pose.pose.orientation.x = quat.x();
  estimated_pose.pose.orientation.y = quat.y();
  estimated_pose.pose.orientation.z = quat.z();
  estimated_pose.pose.orientation.w = quat.w();

  return estimated_pose;
}
#endif
