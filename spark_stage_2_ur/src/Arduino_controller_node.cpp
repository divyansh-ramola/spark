#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include "spark_stage_2_ur/srv/arduino_command.hpp"
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <chrono>
#include <thread>
#include <queue>
#include <mutex>
#include <sstream>

using namespace std::chrono_literals;
using namespace LibSerial;

class ArduinoMotorController : public rclcpp::Node
{
public:
    ArduinoMotorController() : Node("arduino_motor_controller")
    {
        // Declare parameters
        this->declare_parameter("serial_port", "/dev/ttyACM0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("timeout_ms", 30000);

        // Get parameters
        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();
        timeout_ms_ = this->get_parameter("timeout_ms").as_int();

        // Initialize serial connection
        try {
            serial_port_ = std::make_unique<SerialPort>(port);
            
            // Configure serial port
            serial_port_->SetBaudRate(BaudRate::BAUD_115200);
            serial_port_->SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_->SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            serial_port_->SetParity(Parity::PARITY_NONE);
            serial_port_->SetStopBits(StopBits::STOP_BITS_1);
            
            if (serial_port_->IsOpen()) {
                RCLCPP_INFO(this->get_logger(), "Serial port opened: %s at %d baud", 
                           port.c_str(), baud);
                
                // Wait for Arduino to be ready
                std::this_thread::sleep_for(2s);
                serial_port_->FlushInputBuffer();
                
                // Wait for READY message
                waitForReady();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial exception: %s", e.what());
        }

        // Create service
        command_service_ = this->create_service<spark_stage_2_ur::srv::ArduinoCommand>(
            "arduino/send_command",
            std::bind(&ArduinoMotorController::commandCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));

        // Create publisher for Arduino status/logs
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "arduino/status", 10);

        // Create timer for reading serial data
        serial_timer_ = this->create_wall_timer(
            10ms, std::bind(&ArduinoMotorController::serialReadCallback, this));

        RCLCPP_INFO(this->get_logger(), "Arduino Motor Controller Node initialized");
    }

    ~ArduinoMotorController()
    {
        if (serial_port_ && serial_port_->IsOpen()) {
            serial_port_->Close();
        }
    }

private:
    // Serial communication
    std::unique_ptr<SerialPort> serial_port_;
    int timeout_ms_;
    
    // ROS2 components
    rclcpp::Service<spark_stage_2_ur::srv::ArduinoCommand>::SharedPtr command_service_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr serial_timer_;
    
    // Buffer for incoming serial data
    std::string serial_buffer_;

    void waitForReady()
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for Arduino READY signal...");
        auto start = std::chrono::steady_clock::now();
        
        while (rclcpp::ok()) {
            try {
                if (serial_port_->IsDataAvailable()) {
                    std::string line;
                    serial_port_->ReadLine(line, '\n', 1000);
                    
                    // Trim whitespace
                    line.erase(0, line.find_first_not_of(" \n\r\t"));
                    line.erase(line.find_last_not_of(" \n\r\t") + 1);
                    
                    if (!line.empty()) {
                        RCLCPP_INFO(this->get_logger(), "Arduino: %s", line.c_str());
                        
                        if (line == "READY") {
                            RCLCPP_INFO(this->get_logger(), "Arduino is READY!");
                            return;
                        }
                    }
                }
            } catch (const std::exception& e) {
                // Continue waiting
            }
            
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();
            
            if (elapsed > 10) {
                RCLCPP_WARN(this->get_logger(), "Arduino READY timeout, proceeding anyway");
                return;
            }
            
            std::this_thread::sleep_for(100ms);
        }
    }

    void serialReadCallback()
    {
        if (!serial_port_ || !serial_port_->IsOpen()) {
            return;
        }

        try {
            // Check if data is available (like Python's in_waiting)
            while (serial_port_->IsDataAvailable()) {
                std::string line;
                // Read a complete line (blocking with timeout)
                try {
                    serial_port_->ReadLine(line, '\n', 100);
                    
                    // Trim whitespace
                    line.erase(0, line.find_first_not_of(" \n\r\t"));
                    line.erase(line.find_last_not_of(" \n\r\t") + 1);
                    
                    if (line.empty()) {
                        continue;
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "Arduino response: %s", line.c_str());
                    
                    // Publish status message
                    auto msg = std_msgs::msg::String();
                    msg.data = line;
                    status_pub_->publish(msg);

                    // Process response
                    processResponse(line);
                    
                } catch (const LibSerial::ReadTimeout&) {
                    // Timeout reading line - no complete line available yet
                    break;
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", e.what());
        }
    }

    void processResponse(const std::string& response)
    {
        // Just log all Arduino messages for monitoring
        // The actual response handling is done in sendCommandAndWait
        RCLCPP_INFO(this->get_logger(), "Arduino: %s", response.c_str());
    }

    bool sendCommandAndWait(char command, const std::string& command_name)
    {
        if (!serial_port_ || !serial_port_->IsOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open");
            return false;
        }

        // Send command with newline (like Python code does)
        try {
            RCLCPP_INFO(this->get_logger(), "Sending Arduino command: %c", command);
            std::string cmd_str = std::string(1, command) + "\n";
            serial_port_->Write(cmd_str);
            serial_port_->DrainWriteBuffer();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send command: %s", e.what());
            return false;
        }

        // Wait for completion - poll serial port directly like Python does
        auto start = std::chrono::steady_clock::now();
        std::string expected_complete;
        std::string expected_failed;
        
        // Map command to expected responses
        switch(command) {
            case 'h':
                expected_complete = "HOME_COMPLETE";
                expected_failed = "HOME_FAILED";
                break;
            case 'f':
                expected_complete = "FIND_METAL_COMPLETE";
                expected_failed = "FIND_METAL_FAILED";
                break;
            case 'm':
                expected_complete = "MOVE_M0_AWAY_COMPLETE";
                expected_failed = "MOVE_M0_AWAY_FAILED";
                break;
            case 'n':
                expected_complete = "MOVE_M1_AWAY_COMPLETE";
                expected_failed = "MOVE_M1_AWAY_FAILED";
                break;
            case 'p':
                expected_complete = "MOVE_M0_HOME_COMPLETE";
                expected_failed = "MOVE_M0_HOME_FAILED";
                break;
            case 'q':
                expected_complete = "MOVE_M1_HOME_COMPLETE";
                expected_failed = "MOVE_M1_HOME_FAILED";
                break;
            case 'a':
                expected_complete = "MOVE_M1_FORWARD_COMPLETE";
                expected_failed = "MOVE_M1_FORWARD_FAILED";
                break;
            case 'b':
                expected_complete = "MOVE_M1_BACKWARD_COMPLETE";
                expected_failed = "MOVE_M1_BACKWARD_FAILED";
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown command: %c", command);
                return false;
        }
        
        while (rclcpp::ok()) {
            try {
                // Check if data is available (like Python's in_waiting)
                if (serial_port_->IsDataAvailable()) {
                    std::string response;
                    serial_port_->ReadLine(response, '\n', 100);
                    
                    // Trim whitespace
                    response.erase(0, response.find_first_not_of(" \n\r\t"));
                    response.erase(response.find_last_not_of(" \n\r\t") + 1);
                    
                    if (!response.empty()) {
                        RCLCPP_INFO(this->get_logger(), "Arduino response: %s", response.c_str());
                        
                        // Publish all Arduino responses as status
                        auto msg = std_msgs::msg::String();
                        msg.data = response;
                        status_pub_->publish(msg);
                        
                        // Check if command completed
                        if (response == expected_complete) {
                            RCLCPP_INFO(this->get_logger(), 
                                       "Arduino command %c completed successfully", command);
                            return true;
                        }
                        
                        // Check for errors
                        if (response == expected_failed || response.find("ERROR") != std::string::npos) {
                            RCLCPP_ERROR(this->get_logger(), "Arduino error: %s", response.c_str());
                            return false;
                        }
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error reading from Arduino: %s", e.what());
                // Continue trying
            }

            // Check timeout
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - start).count();
            
            if (elapsed > timeout_ms_) {
                RCLCPP_ERROR(this->get_logger(), 
                           "Timeout waiting for Arduino command %c completion", command);
                return false;
            }

            // Small delay to prevent CPU overuse (like Python's time.sleep(0.01))
            std::this_thread::sleep_for(10ms);
        }

        return false;
    }

    void commandCallback(
        const std::shared_ptr<spark_stage_2_ur::srv::ArduinoCommand::Request> request,
        std::shared_ptr<spark_stage_2_ur::srv::ArduinoCommand::Response> response)
    {
        // Validate and extract command character
        if (request->command.empty()) {
            response->success = false;
            response->message = "Command string is empty. Valid commands are: h, f, m, n, p, q, a, b";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        char command = request->command[0];
        
        // Map command to name for logging
        std::string command_name;
        switch(command) {
            case 'h':
            case 'H':
                command_name = "Home Motors";
                command = 'h';
                break;
            case 'f':
            case 'F':
                command_name = "Find Metal";
                command = 'f';
                break;
            case 'm':
            case 'M':
                command_name = "Move M0 Away";
                command = 'm';
                break;
            case 'n':
            case 'N':
                command_name = "Move M1 Away";
                command = 'n';
                break;
            case 'p':
            case 'P':
                command_name = "Move M0 Towards Home";
                command = 'p';
                break;
            case 'q':
            case 'Q':
                command_name = "Move M1 Towards Home";
                command = 'q';
                break;
            case 'a':
            case 'A':
                command_name = "Move M1 Forward";
                command = 'a';
                break;
            case 'b':
            case 'B':
                command_name = "Move M1 Backward";
                command = 'b';
                break;
            default:
                response->success = false;
                response->message = "Invalid command: '" + std::string(1, command) + 
                                   "'. Valid commands are: h, f, m, n, p, q, a, b";
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Arduino command '%c' (%s) service called", 
                   command, command_name.c_str());
        
        bool success = sendCommandAndWait(command, command_name);
        
        response->success = success;
        if (success) {
            response->message = command_name + " completed successfully";
        } else {
            response->message = "Failed to complete " + command_name;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArduinoMotorController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}