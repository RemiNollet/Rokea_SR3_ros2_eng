#include <rclcpp/rclcpp.hpp>

#include <rokae/robot.h>
#include <rokae/motion_control_rt.h>

#include <chrono>
#include <exception>
#include <string>
#include <thread>

class StopMotionDemoNode : public rclcpp::Node
{
public:
	StopMotionDemoNode()
	: Node("stop_motion_demo")
	{
		robot_ip_ = this->declare_parameter<std::string>("robot_ip", "192.168.2.160");
		local_ip_ = this->declare_parameter<std::string>("local_ip", "192.168.2.100");
		power_on_before_stop_ = this->declare_parameter<bool>("power_on_before_stop", true);
		set_idle_after_stop_ = this->declare_parameter<bool>("set_idle_after_stop", false);
		enable_nrt_reset_ = this->declare_parameter<bool>("enable_nrt_reset", false);
	}

	bool run_demo()
	{
		std::error_code ec;

		try {
			rokae::xMateRobot robot(robot_ip_, local_ip_);

			RCLCPP_INFO(this->get_logger(), "Connect the robot: robot_ip=%s local_ip=%s", robot_ip_.c_str(), local_ip_.c_str());
			robot.connectToRobot(ec);
			if (ec) {
				RCLCPP_ERROR(this->get_logger(), "Connection failed: %s", ec.message().c_str());
				return false;
			}

			auto state = robot.operationState(ec);
			if (ec) {
				RCLCPP_WARN(this->get_logger(), "Failed to read operationState: %s", ec.message().c_str());
				ec.clear();
			} else {
				RCLCPP_INFO(this->get_logger(), "CurrentoperationState=%d", static_cast<int>(state));
			}

			if (state == rokae::OperationState::rlProgram) {
				RCLCPP_INFO(this->get_logger(), "RL engineering operation detected, trying firstpauseProject()");
				robot.pauseProject(ec);
				if (ec) {
					RCLCPP_WARN(this->get_logger(), "pauseProject()Return error: %s", ec.message().c_str());
					ec.clear();
				}
			}

			if (power_on_before_stop_) {
				robot.setPowerState(true, ec);
				if (ec) {
					RCLCPP_WARN(this->get_logger(), "Power-on returns an error(Continue executing shutdown): %s", ec.message().c_str());
					ec.clear();
				}
			}

			RCLCPP_INFO(this->get_logger(), "Execute real-time stop(Priority): stopServoJoint() + stopMove()");
			try {
				auto rt_controller = robot.getRtMotionController().lock();
				if (!rt_controller) {
					RCLCPP_WARN(this->get_logger(), "Failed to get real-time controller, skipping real-time stop");
				} else {
					try {
						rt_controller->stopServoJoint();
					} catch (const std::exception &ex) {
						RCLCPP_WARN(this->get_logger(), "stopServoJoint()Abnormal: %s", ex.what());
					} catch (...) {
						RCLCPP_WARN(this->get_logger(), "stopServoJoint()Unknown exception");
					}

					try {
						rt_controller->stopMove();
					} catch (const std::exception &ex) {
						RCLCPP_WARN(this->get_logger(), "stopMove()Abnormal: %s", ex.what());
					} catch (...) {
						RCLCPP_WARN(this->get_logger(), "stopMove()Unknown exception");
					}
				}
			} catch (const std::exception &ex) {
				RCLCPP_WARN(this->get_logger(), "Real-time process stop exception: %s", ex.what());
			} catch (...) {
				RCLCPP_WARN(this->get_logger(), "Unknown exception when stopping the process in real time");
			}

			RCLCPP_INFO(this->get_logger(), "Perform non-real-time stop: stop() And wait until it comes to a complete stop");
			robot.stop(ec);
			if (ec) {
				RCLCPP_WARN(this->get_logger(), "stop()Return error: %s", ec.message().c_str());
				ec.clear();
			}

			if (!wait_until_motion_stopped(robot, std::chrono::milliseconds(3000), std::chrono::milliseconds(100))) {
				RCLCPP_WARN(this->get_logger(), "Waiting for a complete stop timed out, continue tryingmoveReset");
			}

			if (enable_nrt_reset_) {
				std::error_code st_ec;
				auto cur_state = robot.operationState(st_ec);
				if (st_ec) {
					RCLCPP_WARN(this->get_logger(), "Failed to read operationState, skippingmoveReset: %s", st_ec.message().c_str());
				} else if (cur_state == rokae::OperationState::rtControlling) {
					RCLCPP_WARN(this->get_logger(), "Currently still rtControlling, skip moveReset to avoid crashing during mode switching");
				} else {
					RCLCPP_INFO(this->get_logger(), "enable_nrt_reset=true，Start retrymoveReset()");
					bool reset_ok = false;
					for (int attempt = 1; attempt <= 20; ++attempt) {
						robot.moveReset(ec);
						if (!ec) {
							reset_ok = true;
							RCLCPP_INFO(this->get_logger(), "moveReset()Success, attempt=%d", attempt);
							break;
						}

						const std::string msg = ec.message();
						RCLCPP_WARN(this->get_logger(), "moveReset()Failure, attempt=%d, err=%s", attempt, msg.c_str());
						ec.clear();
						std::this_thread::sleep_for(std::chrono::milliseconds(100));
					}

					if (!reset_ok) {
						RCLCPP_WARN(this->get_logger(), "moveReset()Failed after multiple retries, it is recommended to check the current task source of the controller(RL/Teach Pendant / External Control)");
					}
				}
			} else {
				RCLCPP_INFO(this->get_logger(), "Skip by defaultmoveReset()；If execution is required, please set the parameters enable_nrt_reset:=true");
			}

			if (set_idle_after_stop_) {
				robot.setMotionControlMode(rokae::MotionControlMode::Idle, ec);
				if (ec) {
					RCLCPP_WARN(this->get_logger(), "Failed to switch to Idle: %s", ec.message().c_str());
					ec.clear();
				}
			}

			RCLCPP_INFO(this->get_logger(), "Stop motion example execution completed");
			return true;
		}
		catch (const std::exception &ex) {
			RCLCPP_ERROR(this->get_logger(), "Runtime exception: %s", ex.what());
			return false;
		}
		catch (...) {
			RCLCPP_ERROR(this->get_logger(), "Runtime Exception: Unknown Exception");
			return false;
		}
	}

private:
	static bool is_motion_active(rokae::OperationState state)
	{
		return state == rokae::OperationState::moving ||
		       state == rokae::OperationState::jogging ||
		       state == rokae::OperationState::rtControlling ||
		       state == rokae::OperationState::rlProgram ||
		       state == rokae::OperationState::demo ||
		       state == rokae::OperationState::dynamicIdentify ||
		       state == rokae::OperationState::frictionIdentify ||
		       state == rokae::OperationState::loadIdentify;
	}

	template <class RobotT>
	bool wait_until_motion_stopped(RobotT &robot,
						   std::chrono::milliseconds timeout,
						   std::chrono::milliseconds poll_interval)
	{
		auto deadline = std::chrono::steady_clock::now() + timeout;
		while (std::chrono::steady_clock::now() < deadline) {
			std::error_code state_ec;
			auto state = robot.operationState(state_ec);
			if (state_ec) {
				RCLCPP_WARN(this->get_logger(), "Failed to read operationState while waiting for a complete stop: %s", state_ec.message().c_str());
			} else if (!is_motion_active(state)) {
				RCLCPP_INFO(this->get_logger(), "Detected and stopped, operationState=%d", static_cast<int>(state));
				return true;
			}

			std::this_thread::sleep_for(poll_interval);
		}
		return false;
	}

	std::string robot_ip_;
	std::string local_ip_;
	bool power_on_before_stop_;
	bool set_idle_after_stop_;
	bool enable_nrt_reset_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<StopMotionDemoNode>();
	const bool ok = node->run_demo();
	rclcpp::shutdown();
	return ok ? 0 : 1;
}
