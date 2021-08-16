#include <rclcpp/rclcpp.hpp>
#include "autodock_msgs/srv/docking.hpp"
#include "autodock_msgs/msg/current_state.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("docking_client");
  auto client = node->create_client<autodock_msgs::srv::Docking>("autodock_controller/docking_service");

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      printf("docking_client was interrupted while waiting for the service. Exiting.\n");
      return 0;
    }
    printf("service not available, waiting again...\n");
  }

  std::string docking_ = node->declare_parameter("docking", "start");

  auto request = std::make_shared<autodock_msgs::srv::Docking::Request>();
  request->service = docking_;

  auto future_result = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, future_result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    if (future_result.get()->service_success)
      printf("Send service request(docking:=%s) successfully.\n", docking_.c_str());
    else
      printf("Send service request(docking:=%s) failed.\n", docking_.c_str());
  } else {
    printf("docking_client was interrupted. Exiting.\n");
  }

  rclcpp::shutdown();
  return 0;
}
