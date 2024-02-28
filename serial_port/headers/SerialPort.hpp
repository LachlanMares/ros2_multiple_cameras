
#ifndef SERIAL_PORT_SERIAL_PORT_H
#define SERIAL_PORT_SERIAL_PORT_H

#include <string>
#include <fstream> // For file I/O (reading/writing to COM port)
#include <sstream>
#include <vector>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <cstdint>

#include "Exception.hpp"

namespace sp {
    namespace CppLinuxSerial {
        enum class BaudRateType {
            STANDARD,
            CUSTOM,
        };

        enum class BaudRate {
            B_0,
            B_50,
            B_75,
            B_110,
            B_134,
            B_150,
            B_200,
            B_300,
            B_600,
            B_1200,
            B_1800,
            B_2400,
            B_4800,
            B_9600,
            B_19200,
            B_38400,
            B_57600,
            B_115200,
            B_230400,
            B_460800
        };

        enum class NumDataBits {
            FIVE,
            SIX,
            SEVEN,
            EIGHT,
        };

        enum class Parity {
            NONE,
            EVEN,
            ODD,
        };

        enum class NumStopBits {
            ONE,
            TWO,
        };

        enum class HardwareFlowControl {
            OFF,
            ON,
        };

        enum class SoftwareFlowControl {
            OFF,
            ON,
        };

        enum class State {
            CLOSED,
            OPEN,
        };

        class SerialPort {
            public:
                SerialPort();
                SerialPort(const std::string &device, BaudRate baudRate);
                SerialPort(const std::string &device, BaudRate baudRate, NumDataBits numDataBits, Parity parity, NumStopBits numStopBits);
                SerialPort(const std::string &device, BaudRate baudRate, NumDataBits numDataBits, Parity parity, NumStopBits numStopBits, HardwareFlowControl hardwareFlowControl, SoftwareFlowControl softwareFlowControl);
                SerialPort(const std::string &device, speed_t baudRate);
                virtual ~SerialPort();
                void SetDevice(const std::string &device);
                void SetBaudRate(BaudRate baudRate);
                void SetBaudRate(speed_t baudRate);
                void SetNumDataBits(NumDataBits numDataBits);
                void SetParity(Parity parity);
                void SetNumStopBits(NumStopBits numStopBits);
                void SetTimeout(int32_t timeout_ms);
                void SetEcho(bool value);
                void Open();
                void Close();
                void Write(const std::string& data);
                void WriteBinary(const std::vector<uint8_t>& data);
                void Read(std::string& data);
                void ReadBinary(std::vector<uint8_t>& data);
                int32_t Available();
                State GetState();

            private:
                void ConfigureTermios();
                termios2 GetTermios2();
                void SetTermios2(termios2 tty);
                void PortIsOpened(const std::string& prettyFunc);
                
                State state_;
                std::string device_;
                BaudRateType baudRateType_;
                BaudRate baudRateStandard_;
                speed_t baudRateCustom_;
                NumDataBits numDataBits_ = NumDataBits::EIGHT;
                Parity parity_ = Parity::NONE;
                NumStopBits numStopBits_ = NumStopBits::ONE;
                HardwareFlowControl hardwareFlowControl_ = HardwareFlowControl::OFF;
                SoftwareFlowControl softwareFlowControl_ = SoftwareFlowControl::OFF;
                int fileDesc_;
                bool echo_;
                int32_t timeout_ms_;
                std::vector<char> readBuffer_;
                unsigned char readBufferSize_B_;
                static constexpr BaudRate defaultBaudRate_ = BaudRate::B_57600;
                static constexpr int32_t defaultTimeout_ms_ = -1;
                static constexpr unsigned char defaultReadBufferSize_B_ = 255;
            };
    }
}
#endif // #ifndef SERIAL_PORT_SERIAL_PORT_H