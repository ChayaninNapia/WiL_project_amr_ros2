#include "roboteq_ros2_driver/roboteq_ros2_driver.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include <iostream>

#include "std_msgs/msg/string.hpp"


#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>

#define DELTAT(_nowtime, _thentime) ((_thentime > _nowtime) ? ((0xffffffff - _thentime) + _nowtime) : (_nowtime - _thentime))

// Define following to enable cmdvel debug output
#define _CMDVEL_DEBUG

// Define following to enable odom debug output
#define _ODOM_DEBUG

// Define following to publish additional sensor information; comment out to not publish

// #define _ODOM_SENSORS

// Define following to enable service for returning covariance
// #define _ODOM_COVAR_SERVER

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// change to ros2 libraries
// #include <tf2/Math/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

serial::Serial controller;

uint32_t millis()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

namespace Roboteq
{
    Roboteq::Roboteq() : Node("roboteq_ros2_driver")
    {
        pub_odom_tf = this->declare_parameter("pub_odom_tf", false);
        odom_frame = this->declare_parameter("odom_frame", "odom");
        base_frame = this->declare_parameter("base_frame", "base_footprint");
        cmdvel_topic = this->declare_parameter("cmdvel_topic", "cmd_vel");
        odom_topic = this->declare_parameter("odom_topic", "/wheel_odom");
        port = this->declare_parameter("port", "/dev/ttyACM0");
        baud = this->declare_parameter("baud", 115200);
        open_loop = this->declare_parameter("open_loop", false);
        wheel_circumference = this->declare_parameter("wheel_circumference", 0.565486);
        track_width = this->declare_parameter("track_width", 0.815);
        encoder_ppr = this->declare_parameter("encoder_ppr", 50000);
        encoder_cpr = this->declare_parameter("encoder_cpr", 200000);
        max_amps = this->declare_parameter("max_amps", 9.5);
        max_rpm = this->declare_parameter("max_rpm", 300);
        gear_ratio = this->declare_parameter("gear_ratio", 20.0);

        // total_encoder_pulses=0;
        starttime = 0;
        hstimer = 0;
        mstimer = 0;
        odom_idx = 0;
        odom_encoder_toss = 5;
        odom_encoder_left = 0;
        odom_encoder_right = 0;
        odom_x = 0.0;
        odom_y = 0.0;
        odom_yaw = 0.0;
        odom_last_x = 0.0;
        odom_last_y = 0.0;
        odom_last_yaw = 0.0;
        odom_last_time = 0;

        odom_msg = nav_msgs::msg::Odometry();

        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        controller.setPort(port);
        controller.setBaudrate(baud);
        controller.setTimeout(timeout);
        // connect to serial port
        connect();
        // configure motor controller
        cmdvel_setup();
        odom_setup();
        //
        //  odom publisher
        //
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1);
        //
        // cmd_vel subscriber
        //
        cmdvel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            cmdvel_topic, // topic name
            1,            // QoS history depth
            std::bind(&Roboteq::cmdvel_callback, this, std::placeholders::_1));
        using namespace std::chrono_literals;
        // set odometry publishing loop timer at 10Hz
        timer_ = this->create_wall_timer(1ms, std::bind(&Roboteq::run, this));
        odom_baselink_transform_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // enable modifying params at run-time
        param_update_timer =
            this->create_wall_timer(1000ms, std::bind(&Roboteq::update_parameters, this));
    }

    void Roboteq::update_parameters()
    {
        RCLCPP_INFO(this->get_logger(), "Parameters updated ...");
        this->get_parameter("pub_odom_tf", pub_odom_tf);
        this->get_parameter("odom_frame", odom_frame);
        this->get_parameter("base_frame", base_frame);
        this->get_parameter("cmdvel_topic", cmdvel_topic);
        this->get_parameter("odom_topic", odom_topic);
        this->get_parameter("port", port);
        this->get_parameter("baud", baud);
        this->get_parameter("open_loop", open_loop);
        this->get_parameter("wheel_circumference", wheel_circumference);
        this->get_parameter("track_width", track_width);
        this->get_parameter("encoder_ppr", encoder_ppr);
        this->get_parameter("encoder_cpr", encoder_cpr);
        this->get_parameter("max_amps", max_amps);
        this->get_parameter("max_rpm", max_rpm);
        this->get_parameter("gear_ratio", gear_ratio);
    }

    void Roboteq::connect()
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Opening serial port on " << port << " at " << baud << "...");
        try
        {
            controller.open();
            if (controller.isOpen())
            {
                RCLCPP_INFO(this->get_logger(), "Successfully opened serial port");
                return;
            }
        }
        catch (serial::IOException &e)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "serial::IOException: ");
            throw;
        }
        RCLCPP_WARN(this->get_logger(), "Failed to open serial port");
        sleep(5);
    }

    void Roboteq::cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr twist_msg)
    {                                                                                  
        // wheel speed (m/s)
        float right_speed = twist_msg->linear.x + track_width * twist_msg->angular.z / 2.0;
        float left_speed = twist_msg->linear.x - track_width * twist_msg->angular.z / 2.0;

        std::stringstream right_cmd;
        std::stringstream left_cmd;

        // motor speed (rpm)
        int32_t right_rpm = right_speed * gear_ratio / wheel_circumference * 60.0;
        int32_t left_rpm = left_speed * gear_ratio / wheel_circumference * 60.0;

        right_rpm = std::clamp(right_rpm, -static_cast<int32_t>(max_rpm), static_cast<int32_t>(max_rpm));
        left_rpm  = std::clamp(left_rpm,  -static_cast<int32_t>(max_rpm), static_cast<int32_t>(max_rpm));

        right_cmd << "!S 1 " << right_rpm << "\r";
        left_cmd << "!S 2 " << left_rpm << "\r";

// write cmd to motor controller
#ifndef _CMDVEL_FORCE_RUN
        controller.write(right_cmd.str());
        controller.write(left_cmd.str());
        controller.flush();
#endif
    }
    void Roboteq::cmdvel_setup()
    {
        RCLCPP_INFO(this->get_logger(), "configuring motor controller...");

        // stop motors
        controller.write("!G 1 0\r");
        controller.write("!G 2 0\r");
        controller.write("!S 1 0\r");
        controller.write("!S 2 0\r");
        controller.flush();

        // clear break
        controller.write("!DS 0\r");

        //disable echo
        controller.write("^ECHOF 1\r");
        controller.flush();

        // enable watchdog timer (1000 ms)
        controller.write("^RWD 1000\r");
    }

    void Roboteq::cmdvel_loop()
    {
    }

    void Roboteq::cmdvel_run()
    {
#ifdef _CMDVEL_FORCE_RUN
        if (open_loop)
        {
            controller.write("!G 1 100\r");
            controller.write("!G 2 100\r");
        }
        else
        {
            std::stringstream right_cmd;
            std::stringstream left_cmd;
            right_cmd << "!S 1 " << (int)(max_rpm * 0.1) << "\r";
            left_cmd << "!S 2 " << (int)(max_rpm * 0.1) << "\r";
            controller.write(right_cmd.str());
            controller.write(left_cmd.str());
        }
        controller.flush();
#endif
    }

    void Roboteq::odom_setup()
    {
        RCLCPP_INFO(this->get_logger(), "setting up odom...");
        // TODO: implement tf2 broadcaster
        //  RCLCPP_INFO(this->get_logger(), "Broadcasting odom tf"); // might use this-> instead of node
        odom_baselink_transform_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // RCLCPP_INFO_STREAM(get_logger(), "Publishing to topic " << odom_topic);

        // odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);
        // odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 1000)

        // Set up the header
        /*
        tf_msg.header.stamp = 0;
        tf_msg.header.frame_id = odom_frame;
        tf_msg.child_frame_id = base_frame;
        */

        odom_msg.header.stamp = this->get_clock()->now();

        odom_msg.header.frame_id = odom_frame;
        odom_msg.child_frame_id = base_frame;
        /*
            auto message = nav_msgs::msg::Odometry();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "odom";
        message.pose.pose.position.x = new_state.x;
        message.pose.pose.position.y = new_state.y;
        message.pose.pose.orientation.x = quat.x();
        message.pose.pose.orientation.y = quat.y();
        message.pose.pose.orientation.z = quat.z();
        message.pose.pose.orientation.w = quat.w();
        */
        // Set up the pose covariance
        for (size_t i = 0; i < 36; i++)
        {
            odom_msg.pose.covariance[i] = 0;
            odom_msg.twist.covariance[i] = 0;
        }

        odom_msg.pose.covariance[7] = 0.1;
        odom_msg.pose.covariance[14] = 0.1;
        odom_msg.pose.covariance[21] = 0.1;
        odom_msg.pose.covariance[28] = 0.1;
        odom_msg.pose.covariance[35] = 0.1;

        // Set up the twist covariance
        odom_msg.twist.covariance[0] = 0.1;
        odom_msg.twist.covariance[7] = 0.1;
        odom_msg.twist.covariance[14] = 0.1;
        odom_msg.twist.covariance[21] = 0.1;
        odom_msg.twist.covariance[28] = 0.1;
        odom_msg.twist.covariance[35] = 0.1;

        // Set up the transform message: move to odom_publish

        tf2::Quaternion q;
        q.setRPY(0, 0, odom_yaw);

        // tf_msg.transform.translation.x = x;
        // tf_msg.transform.translation.y = y;
        // tf_msg.transform.translation.z = 0.0;
        // tf_msg.transform.rotation.x = q.x();
        // tf_msg.transform.rotation.y = q.y();
        // tf_msg.transform.rotation.z = q.z();
        // tf_msg.transform.rotation.w = q.w();

        // start encoder streaming
        RCLCPP_INFO_STREAM(this->get_logger(), "covariance set");
        RCLCPP_INFO_STREAM(this->get_logger(), "odometry stream starting...");
        odom_stream();

        odom_last_time = millis();
#ifdef _ODOM_SENSORS
        current_last_time = millis();
#endif
    }

    // Odom msg streams
    void Roboteq::odom_stream()
    {

#ifdef _ODOM_SENSORS
        // start encoder and current output (30 hz)
        // doubling frequency since one value is output at each cycle
        //  controller.write("# C_?CR_?BA_# 17\r");
        // start encoder, current and voltage output (30 hz)
        // tripling frequency since one value is output at each cycle
        controller.write("# C_?CR_?BA_?V_# 11\r");
#else
        //  // start encoder output (10 hz)
        //  controller.write("# C_?CR_# 100\r");
        // start encoder output (30 hz)
        
        // auto sending motor feedback
        // controller.write("# C_?F_# 33\r");

        //auto sending motor encoders feedback
        controller.write("# C_?CR_# 33\r");

#endif
        controller.flush();
    }

    void Roboteq::odom_loop()
    {

        uint32_t nowtime = millis();

        // if we haven't received encoder counts in some time then restart streaming
        if (DELTAT(nowtime, odom_last_time) >= 1000)
        {
            odom_stream();
            odom_last_time = nowtime;
        }

        // read sensor data stream from motor controller
        // maybe use while loop to improve cpu usage?
        if (controller.available())
        {
            char ch = 0;
            if (controller.read((uint8_t *)&ch, 1) == 0)
                return;
            if (ch == '\r')
            {
                odom_buf[odom_idx] = 0;
                // CR= is encoder counts
                if (odom_buf[0]=='C' && odom_buf[1]=='R' && odom_buf[2]=='=')
                {
                    //  std::cout<<"ODOM BUFFER"<<odom_buf<<std::endl;
                    unsigned int delim;
                    for (delim = 3; delim < odom_idx; delim++)
                    {
                        if (odom_encoder_toss > 0)
                        {
                            --odom_encoder_toss;
                            break;
                        }
                        if (odom_buf[delim] == ':')
                        {
                            odom_buf[delim] = 0;
                            odom_encoder_right = (int32_t)strtol(odom_buf + 3, NULL, 10);
                            odom_encoder_left = (int32_t)strtol(odom_buf + delim + 1, NULL, 10);

                            odom_publish();
                            break;
                        }
                    }
                }

                odom_idx = 0;
            }
            else if (odom_idx < (sizeof(odom_buf) - 1))
            {
                odom_buf[odom_idx++] = ch;
            }
        }
    }
    void Roboteq::odom_publish()
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        // determine delta time in seconds
        uint32_t nowtime = millis();
        float dt = (float)DELTAT(nowtime, odom_last_time) / 1000.0;
        odom_last_time = nowtime;
        // total_encoder_pulses+=odom_encoder_right;
        // determine deltas of distance and angle

        // float linear = ((float)odom_encoder_right * wheel_circumference / (60 * gear_ratio) + (float)odom_encoder_left * wheel_circumference / (60 * gear_ratio)) / 2;
        // float angular = ((float)odom_encoder_right * wheel_circumference / (60 * gear_ratio) - (float)odom_encoder_left * wheel_circumference / (60 * gear_ratio)) / track_width;

        // Update odometry
        // odom_x += linear * dt * cos(odom_yaw);         // m
        // odom_y += linear * dt * sin(odom_yaw);         // m
        // odom_yaw = NORMALIZE(odom_yaw + angular * dt); // rad

        // // Calculate velocities
        // float vx = (odom_x - odom_last_x) / dt;
        // float vy = (odom_y - odom_last_y) / dt;
        // float vyaw = (odom_yaw - odom_last_yaw) / dt;

        //----------------------------odometry from wheel-encoder-feedback-------------------------------------------------//

        // (1) แปลง counts → ระยะทาง (meters)
        float left_dist  = (float)odom_encoder_left  / (float)encoder_cpr * wheel_circumference;
        float right_dist = (float)odom_encoder_right / (float)encoder_cpr * wheel_circumference;

        // (2) คำนวณระยะ displacement และมุมหมุน
        float linear  = (left_dist + right_dist) / 2.0f;        // [m]
        float angular = (right_dist - left_dist) / track_width; // [rad]

        // (3) อัปเดต pose (linear เป็นระยะทาง จึงไม่ต้องคูณ dt อีก)
        odom_x   += linear * cos(odom_yaw);
        odom_y   += linear * sin(odom_yaw);
        odom_yaw  = NORMALIZE(odom_yaw + angular);

        // (4) ความเร็วเชิงเส้นและเชิงมุม
        float vx   = linear  / dt;
        float vy   = 0.0f;
        float vyaw = angular / dt;

        //-----------------------------------------------------------------------------//

        odom_last_x = odom_x;
        odom_last_y = odom_y;
        odom_last_yaw = odom_yaw;
        // convert yaw to quat;
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(0, 0, odom_yaw);
        // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
        geometry_msgs::msg::Quaternion quat = tf2::toMsg(tf2_quat);

        // tf2::Quaternion quat = tf2::createQuaternionMsgFromYaw(odom_yaw);
        //  TODO: set up tf2_ros

        quat = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), odom_yaw));
        if (pub_odom_tf)
        {
            tf_msg.header.stamp = this->get_clock()->now();
            tf_msg.header.frame_id = odom_frame;
            tf_msg.child_frame_id = base_frame;

            tf_msg.transform.translation.x = odom_x;
            tf_msg.transform.translation.y = odom_y;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = quat;
            odom_baselink_transform_->sendTransform(tf_msg);
        }

        // update odom msg

        // odom_msg->header.seq++; //? not used in ros2 ?
        odom_msg.header.stamp = this->get_clock()->now();

        // delete weird syntax *dt
        odom_msg.pose.pose.position.x = odom_x;
        odom_msg.pose.pose.position.y = odom_y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = quat;
        odom_msg.twist.twist.linear.x = vx;
        
        //correct syntax for ros2
        // odom_msg.twist.twist..y = 0.0;
        // odom_msg.twist.twist..z = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;

        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = vyaw;
        odom_pub->publish(odom_msg);
        // odom_pub.publish(odom_msg); ROS1
    }

    int Roboteq::run()
    {

        // TODO: support automatic re-opening of port after disconnection

        starttime = millis();
        hstimer = starttime;
        mstimer = starttime;
        lstimer = starttime;

        cmdvel_loop();
        odom_loop();
        cmdvel_run();

        return 0;
    }

    Roboteq::~Roboteq()
    {

        if (controller.isOpen())
        {
            controller.close();
        }
        // rclcpp::shutdown(); // uncomment if node doesnt destroy properly
    }

} // end of namespace

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto node = std::make_shared<Roboteq::Roboteq>();
    exec.add_node(node);
    exec.spin();
    printf("stop");
    rclcpp::shutdown();
    // signal(SIGINT, mySigintHandler); // rclcpp::shutdown();
    return 0;
}
