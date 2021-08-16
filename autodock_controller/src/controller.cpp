#include "controller.h"

using namespace automatic_parking ;

void autodock_controller::docking_state_manage(){
    //RCLCPP_INFO(get_logger(),"%s | %s", docking_state.c_str(), last_docking_state.c_str());
    
    if (docking_state == "searching"){
        searching_state_fun();
    }

    if (docking_state == "blind"){
        blind_state_fun();
    }
        
    if (docking_state == "centering"){
        centering_state_fun();
    }
        
    if (docking_state == "approach"){
        approach_state_fun();
    }

    if (docking_state == "final_approach"){
        final_approach_state_fun();
    }

    if (docking_state == "docked"){
        neuron_stop();
    }
}

void autodock_controller::set_docking_state(std::string new_docking_state){
    if (docking_state != new_docking_state){
        last_docking_state = docking_state;
        docking_state = new_docking_state;
        RCLCPP_INFO(get_logger(),"docking state: %s, last state: %s", docking_state.c_str(), last_docking_state.c_str());
    }
}

void autodock_controller::set_action_state(std::string new_action_state){
    if (action_state != new_action_state){
        last_action_state = action_state;
        action_state = new_action_state;
        RCLCPP_INFO(get_logger(),"action state: %s, last state: %s", action_state.c_str(), last_action_state.c_str());
    }
}

void autodock_controller::searching_state_fun(){
    centering_counter = 0;
    if (action_state== "turning"){
        return;
    }
    if (action_state == "jogging"){
        return;
    }
    if (tag_callback_counter<lost_tag_max){
        set_action_state("count_tag_callbacks");
    }
    else{
        tag_callback_counter = 0;
        set_action_state("");
        neuron_turn(default_turn*sign(tag_y));
    }
}

void autodock_controller::blind_state_fun(){
    if (action_state == "turning"){
        return;
    }
    if (action_state == "jogging"){
        return;
    }
    if (in_view){
        neuron_stop();
        neuron_turn(blind_angle*sign(-tag_y));
    }
    else{
        neuron_stop();
        neuron_forward(fabs(tag_y/1.5));
        set_docking_state("searching");
    }
}

void autodock_controller::centering_state_fun(){
    if (action_state == "turning"){
        return;
    }
    if (action_state == "jogging"){
        return;
    }
    if (tag_callback_counter < 1){
        centering_counter += 1;
        //RCLCPP_INFO(get_logger(),"centering_counter:%d" , centering_counter);
        set_action_state("count_tag_callbacks");
        return;
    }
    tag_callback_counter = 0;
    set_action_state("");

    if (centering_counter >= max_center_count){
        RCLCPP_WARN(get_logger(),"centering failed. reverting to last state: searching");
        tag_callback_counter = 0;
        set_docking_state("searching");
        return;
    }
    if (in_view){
        if (fabs(pose_set.theta)>pose_set.theta_bounds){
            neuron_stop();
            neuron_turn(pose_set.theta);
            
        }
        else{
            neuron_stop();
            set_docking_state("approach");  
        }
    }
}

void autodock_controller::approach_state_fun(){
    centering_counter = 0;
    if (in_view){
        approach_counter = 0;
        if (action_state == "jogging"){
            return;
        }
        if (desire_angle == 0){
            if (fabs(pose_set.theta)>pose_set.theta_bounds){
                RCLCPP_INFO(get_logger(),"approach angle exceeded: %f", fabs(pose_set.theta));
                set_docking_state("centering");
            }
            else{
                if (fabs(pose_set.distance-finish_distance) < jog_distance){
                    neuron_stop();
                    neuron_forward(pose_set.distance - finish_distance);
                    set_docking_state("final_approach");
                }
                else{
                    neuron_forward(jog_distance);
                }
            }
        }
        else{
            neuron_forward(jog_distance);
        }
    }
    else{
        approach_counter += 1;
        if(approach_counter > lost_tag_max){
            neuron_stop();
            set_docking_state("searching");
        } 
    }
}

void autodock_controller::final_approach_state_fun(){
    if (action_state == "turning"){
        return;
    }
    if (action_state == "jogging"){
        return;
    }  

    if (in_view and fabs(pose_set.distance-finish_distance)>0.1){
        set_docking_state("approach");
        return;
    }

    if (M_PI-fabs(tag_yaw)>pose_set.theta_bounds){
        neuron_turn((M_PI-fabs(tag_yaw))*sign(-tag_yaw));
        return;
    }
    else{
        neuron_stop();
        set_docking_state("docked");
        RCLCPP_INFO(get_logger(), "Finish Docking!");
    }
}

void autodock_controller::neuron_forward(double distance){
    double cmd_vel_linear ;
    if (action_state == ""){
        set_action_state("jogging");
        
        if (distance > 0){
            cmd_vel_linear = cmd_vel_linear_rate;
        }
        else{
            cmd_vel_linear = -cmd_vel_linear_rate;
        }

        if (fabs(pose_set.distance) < final_approach_distance){
            cmd_vel_linear = cmd_vel_linear/2;
        }
    }
    else{
        cmd_vel_linear = 0;
        return;
    }
    cmd_vel_msg.linear.x = cmd_vel_linear;
    robot_point_temp = robot_point;
    temp_distance = distance;
}

void autodock_controller::neuron_stop(){
    set_action_state("");
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.angular.z = 0;
}

void autodock_controller::neuron_turn(double radians){
    double cmd_vel_angular;
    if (action_state == ""){
        set_action_state("turning");
        if (radians>0){
            cmd_vel_angular = -cmd_vel_angular_rate ;
        }
        else{
            cmd_vel_angular = cmd_vel_angular_rate;
        }
        if (fabs(radians) < 0.1){
            cmd_vel_angular = cmd_vel_angular/2;
        }
    }
    else{
        cmd_vel_angular = 0;
        return;
    }
    cmd_vel_msg.angular.z = cmd_vel_angular;
    robot_point_temp = robot_point;
    temp_theta = radians;
}

void autodock_controller::action_state_manage(){
    //RCLCPP_INFO(get_logger(), "%s | %s", action_state.c_str(), last_action_state.c_str());
    if (action_state == "jogging"){
        if (distance(robot_point_temp, robot_point) >= fabs(temp_distance)){
            neuron_stop();
            if (docking_state == "approach"){set_docking_state("centering");}
        }
    }
    if (action_state == "turning" ){
        if (fabs(robot_point_temp[2]-robot_point[2])>= fabs(temp_theta)){neuron_stop();}
        else if (fabs(pose_set.theta) < pose_set.theta_bounds and docking_state != "blind" and docking_state != "final_approach"){
            neuron_stop();}
    }
    if (tag_y){
        if ((fabs(tag_y)<0.02) and (desire_angle == tune_angle)){
            neuron_stop();
            final_counter += 1;
            if (final_counter > 3){
                desire_angle = 0;
            }
        }
    }
    else{
       desire_angle = tune_angle;
    }

    if (docking_state != "") vel_pub->publish(cmd_vel_msg);
}

void autodock_controller::tags_callback(){
    if (action_state == "count_tag_callbacks"){
        tag_callback_counter += 1;
    }
    else{
        tag_callback_counter = 0;
    }

    if (docking_state=="searching" and in_view){
        if(fabs(tag_y/tag_x) >= fabs(tag_x/2)){
            neuron_stop();
            tag_callback_counter = 0;
            set_docking_state("blind");
        }
        else{
            neuron_stop();
            tag_callback_counter = 0;
            set_docking_state("centering");
        }
    }   
}

void autodock_controller::fid2pos(){
    double x_trans = tf_bot2dock.transform.translation.x;
    double y_trans = tf_bot2dock.transform.translation.y;
    double theta = atan2(-y_trans, x_trans);
    double r = sqrt(pow(x_trans ,2) + pow(y_trans , 2));
    double theta_bounds;

    if (r > 3.0){theta_bounds = approach_angle;}
    else{theta_bounds = r/30.0;}

    //RCLCPP_INFO(get_logger(),"Theta: %3.3f, distance: %3.3f, theta_bounds: %3.3f", theta, r, theta_bounds);
    pose_set = {theta-desire_angle*sign(tag_y), r, theta_bounds};
}

void autodock_controller::transform_filter(geometry_msgs::msg::TransformStamped &tf_){
    double time = tf_.header.stamp.sec + double(tf_dock2bot.header.stamp.nanosec)*(1e-9);
    if (time == last_time){
        in_view = false;
    }
    else{in_view = true;}
    last_time = time;
}

void autodock_controller::receive_tf(){
    try{
        tf_odom = buffer_->lookupTransform("odom","base_link",tf2::TimePointZero);
        odom_x = tf_odom.transform.translation.x;
        odom_y = tf_odom.transform.translation.y; 
        odom_yaw = tf2::getYaw(tf_odom.transform.rotation);
        robot_point = {odom_x , odom_y , odom_yaw};

        tf_dock2bot = buffer_->lookupTransform(tag_frame, "base_link", tf2::TimePointZero);
        tf_bot2dock = buffer_->lookupTransform("base_link", tag_frame, tf2::TimePointZero);
        transform_filter(tf_dock2bot);
        if (!in_view){
            //RCLCPP_WARN(get_logger(),"Tag Detection Lost");
            return;
        }
        tag_x = tf_dock2bot.transform.translation.x;
        tag_y = tf_dock2bot.transform.translation.y;
        tag_yaw = tf2::getYaw(tf_dock2bot.transform.rotation);
        fid2pos();
    }
    catch(tf2::TransformException &ex){
        RCLCPP_ERROR(get_logger(),"%s",ex.what());
        in_view = false;
    }
}

void autodock_controller::run(){
    receive_tf();
    tags_callback();
    docking_state_manage();
    action_state_manage();
    state_publish();
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<automatic_parking::autodock_controller>() ;
    rclcpp::Rate rate(30.0);
    controller_node->set_docking_state("");
    while (rclcpp::ok()){
        controller_node->run();
        rclcpp::spin_some(controller_node);
        rate.sleep();
    }
    return 0;
}