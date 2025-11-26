#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>	// the include of the new message

using namespace std::chrono_literals;

class ForceLand : public rclcpp::Node
{
	public:
	ForceLand() : Node("force_land"), need_land(false)
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
		qos, std::bind(&ForceLand::height_callback, this, std::placeholders::_1));

		// new subscriber for the VehicleLandDetected.msg
		landing_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
		qos, std::bind(&ForceLand::land_callback, this, std::placeholders::_1));

		publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

		timer_ = this->create_wall_timer(10ms, std::bind(&ForceLand::activate_switch, this));
	}

	private:
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;

	// definition of the new subscription
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr landing_subscription_;

	rclcpp::TimerBase::SharedPtr timer_;

	bool need_land;
	bool land_detected = true;			// new boolean variable to track landing
	bool procedure_activated = false;	// tracking the activation of the landing procedure

	void height_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) 
	{
		float z_ = -msg->z;
		std::cout << "Current drone height: " << z_ << " meters" <<  std::endl;
		if(z_ > 20) 
		{
			if(land_detected){
				need_land = true;
				procedure_activated = true;
			}	
		}else{
			if(procedure_activated){
				procedure_activated = false;
				land_detected = false;
			}
		}

		return;
	}
	
	void land_callback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg) 
	{
		if(msg->landed)
		{
			land_detected = true;
			std::cout << "The drone has landed!" <<  std::endl;
		}

		return;
	}

	void activate_switch()
	{
		if(need_land)
		{
			std::cout << "Drone height exceeded 20 meters threshold, Landing forced" << std::endl;
			auto command = px4_msgs::msg::VehicleCommand();
			command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
			this->publisher_->publish(command);
			need_land = false;
		}
	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_local_position listener node..." << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ForceLand>());
	rclcpp::shutdown();
	return 0;
}