/*
Author:
    Lachlan Mares, lachlan.mares@gmail.com

License:
    GPL-3.0

Description:

*/

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include "SerialPort.hpp"

using namespace sp;

class SerialPortTester : public rclcpp::Node {
    public:
        SerialPortTester() : Node("serial_port_test_node") {
            SerialPort serialPort = SerialPort("/dev/ttyACM0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
            serialPort.Open();
        }

    private:
        
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialPortTester>();

    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}