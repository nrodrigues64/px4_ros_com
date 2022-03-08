/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/takeoff_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_test") {

		takeoff_status_sub_ = this->create_subscription<px4_msgs::msg::TakeoffStatus>(
			"fmu/takeoff_status/out",

            10,

			[this](const px4_msgs::msg::TakeoffStatus::UniquePtr msg) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED TAKEOFF STATUS DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "ts: "          << msg->timestamp    << std::endl;	
			switch(msg->takeoff_state){
				case 0:
					std::cout << "takeoff_state: UNITIALIZED" << std::endl;
					break;
				case 1:
					std::cout << "takeoff_state: DISARMED" << std::endl;
					break;
				case 2:
					std::cout << "takeoff_state: SPOOLUP" << std::endl;
					break;
				case 3:
					std::cout << "takeoff_state: READY FOR TAKEOFF" << std::endl;
					break;
				case 4:
					std::cout << "takeoff_state: RAMPUP" << std::endl;
					break;
				case 5:
					std::cout << "takeoff_state: FLIGHT" << std::endl;
					break;
				default:
					std::cout << "takeoff_state: default" << std::endl;
					break;
			};
			if(msg->takeoff_state == 3){
				this->takeoff();
			} else if (msg->takeoff_state == 1){
				this->arm();
			}
			this->takeoff_state = msg->takeoff_state;
		});

		/*vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			"fmu/vehicle_odometry/out",
			10,
			[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
				std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
				std::cout << "RECEIVED Vehicle ODOMETRY DATA"   << std::endl;
				std::cout << "============================="   << std::endl;
				std::cout << "x: " << msg->x << std::endl;
				std::cout << "y: " << msg->y << std::endl;
				std::cout << "z: " << msg->z << std::endl;


			}
		);*/

		vehicle_control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
			"fmu/vehicle_control_mode/out",
			10,
			[this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg) {
				std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
				std::cout << "RECEIVED Vehicle ODOMETRY DATA"   << std::endl;
				std::cout << "============================="   << std::endl;
				if(msg->flag_control_offboard_enabled){
					std::cout << "true" << std::endl;
				} else std::cout << "false" << std::endl;


			}
		);

		/*vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
			"fmu/vehicle_status/out",

            10,

			[this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED VEHICLE STATUS DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "ts: "          << msg->timestamp    << std::endl;	
			switch(msg->arming_state){
				case 0:
					std::cout << "arming_state: INIT" << std::endl;
					break;
				case 1:
					std::cout << "arming_state: STANDBY" << std::endl;
					break;
				case 2:
					std::cout << "arming_state: ARMED" << std::endl;
					break;
				case 3:
					std::cout << "arming_state: ERROR" << std::endl;
					break;
				case 4:
					std::cout << "arming_state: SHUTDOWN" << std::endl;
					break;
				case 5:
					std::cout << "arming_state: AIR RESTORE" << std::endl;
					break;
				case 6:
					std::cout << "arming_state: MAX" << std::endl;
					break;
				default:
					std::cout << "arming_state: default" << std::endl;
					break;
			};
		});*/

		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in",10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in",10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in",10);
		

		
		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

		offboard_setpoint_counter_ = 0;
		
		auto timer_callback = [this]() -> void {
			
			if (offboard_setpoint_counter_ == 10) {
				
				if(this->takeoff_state == 1) {
					publish_trajectory_setpoint();
					// Change to Offboard mode after 10 setpoints
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				}
				

				// Arm the vehicle
				this->arm();
				
				
			}
            		// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();
		
				 // stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm() const;
	void disarm() const;
	void takeoff() const;
private:
	unsigned int takeoff_state = 100; //To store and utilize the takeoff_status out of the subscriber 
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::TakeoffStatus>::SharedPtr takeoff_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_sub_;
	//rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0, float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0) const;
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief  Send a command to TakeOff the vehicle
 * 
 */
void OffboardControl::takeoff() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0,0,0, 2, 0,0, 0);
	//VEHICLE_CMD_NAV_VTOL_TAKEOFF		
	RCLCPP_INFO(this->get_logger(), "Takeoff cmd send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint() const {
	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.x = 0.0;
	msg.y = 0.0;
	msg.z = -5.0;
	msg.yaw = 3.14; // [-PI:PI]

	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command,float param1, float param2, float param3, float param4, float param5, float param6, float param7) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
	msg.param7 = param7;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
