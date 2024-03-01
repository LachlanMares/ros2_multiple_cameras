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
using namespace std::chrono_literals;

class SerialPortTester : public rclcpp::Node {
    public:
        SerialPortTester() : Node("serial_port_test_node") {
            SerialPort serialPort = SerialPort("/dev/ttyUSB0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
            serialPort.SetTimeout(1000);

            if (!serialPort.Open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");

            } else {

                std::this_thread::sleep_for(100ms);

                // std::thread t1([&]() {
                //     // Do Something
                //     for (int x = 0; x < 10; x++) {
                //         std::cout << "Reading" << std::endl;
                //         std::string readData;
                //         while (serialPort.Available() <= 1) {
                //             std::this_thread::sleep_for(10ms);
                //         }
                //         serialPort.Read(readData);
                //         std::cout << "readData: " << readData << std::endl;
                //     }
                // });

                // std::thread t2([&]() {
                //     // Do Something
                //     std::this_thread::sleep_for(100ms);
                //     for (int x = 0; x < 10; x++) {
                //         std::this_thread::sleep_for(100ms);
                //         std::cout << "Writing \"Hello\"" << std::endl;
                //         serialPort.Write("Hello");
                //         serialPort.WriteBinaryAT({0, 0});
                //     }
                // });

                std::thread t1([&]() {
                    // Do Something
                    for (int x = 0; x < 10; x++) {
                        std::cout << "Reading" << std::endl;
                        std::vector<uint8_t> readData;
                        serialPort.ReadBinaryAT(readData);
                        std::string dataString = "";

                        for (uint32_t i=0; i<readData.size(); i++) {
                            dataString += std::to_string(readData[i]) + " ";
                        }
                        std::cout << "readData: " << dataString << std::endl;
                    }
                });

                std::thread t2([&]() {
                    // Do Something
                    std::this_thread::sleep_for(100ms);
                    for (int x = 0; x < 10; x++) {
                        std::this_thread::sleep_for(100ms);
                        std::cout << "Writing \"0 1 2 3\"" << std::endl;
                        serialPort.WriteBinaryAT({0, 1, 2, 3});
                    }
                });

                t1.join();
                t2.join();
                
                serialPort.Close();
            }
        }

    private:
        
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialPortTester>();

    // rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}