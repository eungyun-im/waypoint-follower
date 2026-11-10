#include <waypoint_follower/waypointFollower.h>
#include <waypoint_follower/new_platform_control.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto wf = std::make_shared<WaypointFollower>();
  PlatformConnector pc(wf.get());
  wf->setPlatformConnector(&pc);

  rclcpp::Rate loop_rate(20);
  while (rclcpp::ok()) {
    rclcpp::spin_some(wf);
    wf->process();
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
