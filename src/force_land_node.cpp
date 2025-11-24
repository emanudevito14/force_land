#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>

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
        
		manualcontrolsub_ = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>("/fmu/out/manual_control_setpoint",
		qos,std::bind(&ForceLand::use_manual_control,this,std::placeholders::_1));

		landsub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",qos,
		std::bind(&ForceLand::verify_landed,this,std::placeholders::_1));

		statussub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status",qos,
		std::bind(&ForceLand::status_armed,this,std::placeholders::_1));

		publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

		timer_ = this->create_wall_timer(10ms, std::bind(&ForceLand::activate_switch, this));
	}

	private:
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
	rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manualcontrolsub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr landsub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr statussub_;

	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;

	rclcpp::TimerBase::SharedPtr timer_;

	bool need_land;
	bool first_flight=true;
	bool retake_control=false;
	

	void height_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) 
	{
		float z_ = -msg->z;
		std::cout << "Current drone height: " << z_ << " meters" <<  std::endl;
		
        if(retake_control == true && first_flight == false)
		{
			return;
		}
		if(z_ > 20)
		{
			need_land = true;
			first_flight=false;
		}

		return;
	}
	void use_manual_control(const px4_msgs::msg::ManualControlSetpoint::UniquePtr msg)
	{
		float throttle_= msg->throttle;
		if(throttle_>0.1)
		{
          retake_control=true;
		}
		
	}
	void verify_landed(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg)
	{
		bool landed_= msg->landed;
		if(landed_==true)
		{
			need_land=false;
			first_flight=true;
			retake_control=false;
		}

	}
	void status_armed(const px4_msgs::msg::VehicleStatus::UniquePtr msg)
	{
		auto arming_state_ = msg->arming_state;

		if(arming_state_ == msg->ARMING_STATE_DISARMED)
		{
			need_land=false;
			first_flight=true;
			retake_control=false;
			
		}
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