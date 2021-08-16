#include <controller.h>

using namespace automatic_parking;

void autodock_controller::handle_service(
    const std::shared_ptr < rmw_request_id_t >,
    const std::shared_ptr < autodock_msgs::srv::Docking::Request > request,
    std::shared_ptr < autodock_msgs::srv::Docking::Response >      response)
{
    if (docking_state == "docked") set_docking_state("");

    if ((request->service == "start") && (docking_state == "")) {
        RCLCPP_INFO(this->get_logger(), "dock start");
        set_docking_state("searching");
        response->service_success = true;
    } else if (request->service == "cancel") {
        neuron_stop();
        set_docking_state("");
        response->service_success = true;
    } else {
        response->service_success = false;
        RCLCPP_INFO(this->get_logger(), "Unknown action to autodocking service.");
    }
}

void autodock_controller::state_publish()
{
    autodock_msgs::msg::CurrentState current_state;

    current_state.docking_state = docking_state;
    current_state.action_state = action_state;
    state_pub->publish(current_state);
}
