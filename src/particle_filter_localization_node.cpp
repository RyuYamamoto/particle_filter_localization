#include "particle_filter_localization/particle_filter_localization.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto particle_filter_localization = std::make_shared<ParticleFilterLocalization>();
  rclcpp::spin(particle_filter_localization);
  rclcpp::shutdown();
  return 0;
}
