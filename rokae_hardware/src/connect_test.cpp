#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <atomic>
#include <iomanip>
#include "rokae/robot.h"

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cout << "Usage: " << argv[0] << " <RobotIP> <LocalIP>" << std::endl;
        return 1;
    }
    
    std::string robot_ip = argv[1];
    std::string local_ip = argv[2];
    
    std::cout << "Rokae Network communication testing\n";
    std::cout << "RobotIP: " << robot_ip << ", LocalIP: " << local_ip << "\n";
    std::cout << "Press Enter to start the test..." << std::endl;
    std::cin.ignore();
    
    try {
        // Connect the robot
        rokae::xMateRobot robot(robot_ip, local_ip);
        std::error_code ec;
        robot.connectToRobot(ec);
        if (ec) throw std::runtime_error("Connection failed: " + ec.message());
        
        // Set robot mode
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
        robot.setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        robot.setPowerState(true, ec);
        if (ec) throw std::runtime_error("Failed to set mode: " + ec.message());
        
        // Move to zero position
        std::array<double, 6> zero_position = {0, 0, 0, 0, 0, 0};
        std::cout << "Move to zero position..." << std::endl;
        auto rtCon = robot.getRtMotionController().lock();
        if (rtCon) {
            rtCon->MoveJ(0.3, robot.jointPos(ec), zero_position);
        }
        
        // Startup State Reception
        robot.startReceiveRobotState(std::chrono::milliseconds(1), {
            rokae::RtSupportedFields::jointPos_m
        });
        
        // Test parameters
        const int TEST_DURATION_MS = 10000;  // 10second
        const int TARGET_FREQ = 1000;        // 1000Hz
        const int CYCLE_US = 1000000 / TARGET_FREQ;  // 1000us = 1ms
        
        // statistical variable
        std::atomic<int> cycle_count{0};
        std::atomic<int> success_count{0};
        std::atomic<int> failed_count{0};
        std::atomic<double> min_delay{10000.0};
        std::atomic<double> max_delay{0.0};
        std::atomic<double> total_delay{0.0};
        
        std::cout << "Start " << TARGET_FREQ << "Hz Network test..." << std::endl;
        
        auto test_start = std::chrono::steady_clock::now();
        auto test_end = test_start + std::chrono::milliseconds(TEST_DURATION_MS);
        
        while (std::chrono::steady_clock::now() < test_end) {
            auto cycle_start = std::chrono::high_resolution_clock::now();
            
            try {
                // Core Test: Reading Joint Positions
                std::array<double, 6> pos{};
                auto read_start = std::chrono::high_resolution_clock::now();
                int ret_=robot.updateRobotState(std::chrono::milliseconds(1)); 
                int ret = robot.getStateData(rokae::RtSupportedFields::jointPos_m, pos);
                auto read_end = std::chrono::high_resolution_clock::now();
                
                // Computation delay
                double delay_ms = std::chrono::duration<double, std::milli>(
                    read_end - read_start).count();
                
                bool success = ((ret_ > 0)&&(ret ==0));
                
                cycle_count++;
                if (success) {
                    success_count++;
                    
                    // Update Delay Statistics
                    if (delay_ms < min_delay) min_delay = delay_ms;
                    if (delay_ms > max_delay) max_delay = delay_ms;
                    total_delay.store(total_delay.load() + delay_ms);
                } else {
                    failed_count++;
                }
                
                // Print once every 100 cycles
                if (cycle_count % 100 == 0) {
                    double success_rate = (double)success_count / cycle_count;
                    std::cout << "#" << cycle_count 
                              << " Success rate: " << std::fixed << std::setprecision(1) << (success_rate * 100) << "%"
                              << " Delay: " << std::setprecision(3) << delay_ms << "ms"
                              << std::endl;
                }
                
            } catch (...) {
                failed_count++;
                cycle_count++;
            }
            
            // Precise frequency control
            auto cycle_end = std::chrono::high_resolution_clock::now();
            auto cycle_time = std::chrono::duration_cast<std::chrono::microseconds>(
                cycle_end - cycle_start);
            
            int wait_us = CYCLE_US - cycle_time.count();
            if (wait_us > 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(wait_us));
            }
        }
        
        // Test ended, calculate statistics
        int total = cycle_count;
        int success = success_count;
        int lost = failed_count;
        
        double avg_delay = (success > 0) ? total_delay / success : 0.0;
        double success_rate = (total > 0) ? (double)success / total : 0.0;
        
        // Output result
        std::cout << "\n\n================ Test Results ================\n";
        std::cout << "Total number of cycles: " << total << "\n";
        std::cout << "Number of successes: " << success << "\n";
        std::cout << "Number of Losses: " << lost << "\n";
        std::cout << "Packet loss rate: " << std::fixed << std::setprecision(2) 
                  << ((double)lost / total * 100) << "%\n";
        std::cout << "Success rate: " << std::fixed << std::setprecision(2) 
                  << (success_rate * 100) << "%\n";
        std::cout << "Minimal latency: " << std::fixed << std::setprecision(3) 
                  << min_delay << "ms\n";
        std::cout << "Average latency: " << std::fixed << std::setprecision(3) 
                  << avg_delay << "ms\n";
        std::cout << "Maximum delay: " << std::fixed << std::setprecision(3) 
                  << max_delay << "ms\n";
        std::cout << "========================================\n";
        
        // Clean
        robot.setPowerState(false, ec);
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
