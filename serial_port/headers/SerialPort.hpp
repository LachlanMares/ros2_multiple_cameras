#include <string>
#include <fstream>    
#include <sstream>
#include <vector>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdio.h>      
#include <unistd.h>     
#include <fcntl.h>      
#include <errno.h>      
#include <system_error>	
#include <sys/ioctl.h>  
#include <cassert>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <algorithm>
#include <iterator>

namespace sp {
    enum BaudRateType {
        STANDARD = 0,
        CUSTOM = 1,
    };

    enum BaudRate {
        B_0 = 0,
        B_50 = 50,
        B_75 = 75,
        B_110 = 110,
        B_134 = 134,
        B_150 = 150,
        B_200 = 200,
        B_300 = 300,
        B_600 = 600,
        B_1200 = 1200,
        B_1800 = 1800,
        B_2400 = 2400,
        B_4800 = 4800,
        B_9600 = 9600,
        B_19200 = 19200,
        B_38400 = 38400,
        B_57600 = 57600,
        B_115200 = 115200,
        B_230400 = 230400,
        B_460800 = 460800,
        B_CUSTOM = -1,
    };

    enum NumDataBits {
        FIVE = 5,
        SIX = 6,
        SEVEN = 7,
        EIGHT = 8,
    };

    enum Parity {
        NONE = 0,
        EVEN = 1,
        ODD = 2,
    };

    enum NumStopBits {
        ONE = 1,
        TWO = 2,
    };

    enum HardwareFlowControl {
        HW_OFF = false,
        HW_ON = true,
    };

    enum SoftwareFlowControl {
        SW_OFF = false,
        SW_ON = true,
    };

    enum State {
        CLOSED = false,
        OPEN = true,
    };

    class SerialPort {
        public:
            SerialPort(std::string device, int baudRate) {
                device_ = device;
                baudRateType_ = GetBaudRateType(baudRate);
                baudRate_ = baudRate;
                timeout_ms_ = defaultTimeout_ms_;
                readBufferSize_B_ = defaultReadBufferSize_B_;
                readBuffer_.reserve(readBufferSize_B_);
                state_ = State::CLOSED;
            };

            SerialPort(std::string device, int baudRate, int numDataBits, int parity, int numStopBits) {
                device_ = device;
                baudRateType_ = GetBaudRateType(baudRate);
                baudRate_ = baudRate;
                numDataBits_ = numDataBits;
                parity_ = parity;
                numStopBits_ = numStopBits;
                timeout_ms_ = defaultTimeout_ms_;
                readBufferSize_B_ = defaultReadBufferSize_B_;
                readBuffer_.reserve(readBufferSize_B_);
                state_ = State::CLOSED;
            };

            SerialPort(std::string device, int baudRate, int numDataBits, int parity, int numStopBits, bool hardwareFlowControl, bool softwareFlowControl) {
                device_ = device;
                baudRateType_ = GetBaudRateType(baudRate);
                baudRate_ = baudRate;
                numDataBits_ = numDataBits;
                parity_ = parity;
                numStopBits_ = numStopBits;
                hardwareFlowControl_ = hardwareFlowControl;
                softwareFlowControl_ = softwareFlowControl;
                timeout_ms_ = defaultTimeout_ms_;
                readBufferSize_B_ = defaultReadBufferSize_B_;
                readBuffer_.reserve(readBufferSize_B_);
                state_ = State::CLOSED;
            };

            ~SerialPort() {
                try {
                    Close();
                } catch(...) {

                }
            }

            void SetDevice(std::string device) {
                device_ = device;
                if(state_ == State::OPEN)
                    ConfigureTermios();
            };

            void SetBaudRate(int baudRate) {
                baudRateType_ = GetBaudRateType(baudRate);
                baudRate_ = baudRate;
                if(state_ == State::OPEN)
                    ConfigureTermios();
            };

            void SetNumDataBits(int numDataBits) {
                numDataBits_ = numDataBits;
                if(state_ == State::OPEN)
                    ConfigureTermios();
            };

            void SetParity(int parity) {
                parity_ = parity;
                if(state_ == State::OPEN)
                    ConfigureTermios();
            };

            void SetNumStopBits(int numStopBits) {
                numStopBits_ = numStopBits;
                if(state_ == State::OPEN)
                    ConfigureTermios();
            };

            bool Open()
            {
                bool retVal = false;
                if(device_.empty()) {
                    std::cout << "Attempted to open file when file path has not been assigned to." << std::endl; 

                } else {
                    // O_RDONLY for read-only, O_WRONLY for write only, O_RDWR for both read/write access
                    fileDesc_ = open(device_.c_str(), O_RDWR);

                    // Check status
                    if(fileDesc_ == -1) {
                        std::cout << "Could not open device " << device_ << ". Is the device name correct and do you have read/write permissions?" << std::endl;

                    } else {
                        ConfigureTermios();
                        state_ = State::OPEN;
                        retVal = true;
                    }
                }
                return retVal;
            };

            void Close() {
                if(fileDesc_ != -1) {
                    auto retVal = close(fileDesc_);
                    if(retVal != 0) {
                        std::cout << "Tried to close serial port " << device_ << ", but close() failed." << std::endl;
                    }
                    fileDesc_ = -1;
                }
                state_ = State::CLOSED;
            };

            void SetEcho(bool value) {
                echo_ = value;
                ConfigureTermios();
            };

            void SetTimeout(int32_t timeout_ms) {
                if(timeout_ms < -1)
                    std::cout << "timeout_ms provided to " << device_ << " was < -1, which is invalid." << std::endl;

                if(timeout_ms > 25500)
                    std::cout << "timeout_ms provided to " << device_ << " was > 25500, which is invalid." << std::endl;

                if(state_ == State::OPEN)
                    std::cout << device_ << " called while state == OPEN." << std::endl;

                timeout_ms_ = timeout_ms;
            };

            int32_t Available() {
                if (PortIsOpened()) {
                    int32_t ret = 0;
                    ioctl(fileDesc_, FIONREAD, &ret);
                    return ret;
                }
            };
  
            bool GetState() {
                return state_;
            };

            void Write(const std::string& data) {
                if (PortIsOpened()) {

                    int writeResult = write(fileDesc_, data.c_str(), data.size());

                    // Check status
                    if (writeResult == -1) {
                        throw std::system_error(EFAULT, std::system_category());
                    }
                }
            };

            void WriteBinary(const std::vector<uint8_t>& data) {
                if (PortIsOpened()) {

                    int writeResult = write(fileDesc_, data.data(), data.size());

                    // Check status
                    if (writeResult == -1) {
                        throw std::system_error(EFAULT, std::system_category());
                    }
                }
            };

            void Read(std::string& data) {
                if (PortIsOpened()) {

                    ssize_t n = read(fileDesc_, &readBuffer_[0], readBufferSize_B_);

                    // Error Handling
                    if(n < 0) {
                        // Read was unsuccessful
                        throw std::system_error(EFAULT, std::system_category());
                    
                    } else if(n == 0) {
                        // n == 0 means EOS, but also returned on device disconnection. We try to get termios2 to distinguish two these two states
                        struct termios2 term2;
                        int rv = ioctl(fileDesc_, TCGETS2, &term2);

                        if(rv != 0) {
                            throw std::system_error(EFAULT, std::system_category());
                        }
                    
                    } else if(n > 0) {
                        data += std::string(&readBuffer_[0], n);
                    }
                }
            };

            void ReadBinary(std::vector<uint8_t>& data) {
                if (PortIsOpened()) {
                    ssize_t n = read(fileDesc_, &readBuffer_[0], readBufferSize_B_);

                    // Error Handling
                    if(n < 0) {
                        // Read was unsuccessful
                        throw std::system_error(EFAULT, std::system_category());

                    } else if(n == 0) {
                        // n == 0 means EOS, but also returned on device disconnection. We try to get termios2 to distinguish two these two states
                        struct termios2 term2;
                        int rv = ioctl(fileDesc_, TCGETS2, &term2);

                        if(rv != 0) {
                            throw std::system_error(EFAULT, std::system_category());
                        }

                    } else if(n > 0) {
                        std::copy(readBuffer_.begin(), readBuffer_.begin() + n, back_inserter(data));
                    }
                }
            };
    
        private:
            bool state_ = false;
            bool echo_ = false;
            bool hardwareFlowControl_ = HardwareFlowControl::HW_OFF;
            bool softwareFlowControl_ = SoftwareFlowControl::SW_OFF;

            std::string device_;

            int baudRateType_ = BaudRateType::STANDARD;
            int baudRate_ = BaudRate::B_57600;
            int numDataBits_ = NumDataBits::EIGHT;
            int parity_ = Parity::NONE;
            int numStopBits_ = NumStopBits::ONE;
            int fileDesc_;
    
            int32_t timeout_ms_;
    
            std::vector<char> readBuffer_;

            unsigned char readBufferSize_B_;
            static constexpr int defaultBaudRate_ = BaudRate::B_57600;
            static constexpr int32_t defaultTimeout_ms_ = -1;
            static constexpr unsigned char defaultReadBufferSize_B_ = 255;

            void ConfigureTermios()
            {
                termios2 tty = GetTermios2();

                // Data bits
                tty.c_cflag &= ~CSIZE;  // CSIZE is a mask for the number of bits per character

                switch(numDataBits_) {
                    case NumDataBits::FIVE:
                        tty.c_cflag = CS5;
                        break;
                    case NumDataBits::SIX:
                        tty.c_cflag |= CS6;
                        break;
                    case NumDataBits::SEVEN:
                        tty.c_cflag |= CS7;
                        break;
                    case NumDataBits::EIGHT:
                        tty.c_cflag |= CS8;
                        break;
                    default:
                        std::cout << "numDataBits value not supported!" << std::endl;
                }

                // Parity 
                switch(parity_) {
                    case Parity::NONE:
                        tty.c_cflag &= ~PARENB;
                        break;
                    case Parity::EVEN:	
                        tty.c_cflag |= PARENB;
                        tty.c_cflag	&= ~PARODD; // Clearing PARODD makes the parity even
                        break;
                    case Parity::ODD:
                        tty.c_cflag |= PARENB;
                        tty.c_cflag	|= PARODD;
                        break;
                    default:
                        std::cout << "parity value not supported!" << std::endl;
                }

                // Stop bits
                switch(numStopBits_) {
                    case NumStopBits::ONE:
                        tty.c_cflag &= ~CSTOPB;
                        break;
                    case NumStopBits::TWO:
                        tty.c_cflag |= CSTOPB;
                        break;
                    default:
                        std::cout << "numStopBits value not supported!" << std::endl;
                }

                // Flow control
                if (hardwareFlowControl_ == HardwareFlowControl::HW_OFF){
                    tty.c_cflag &= ~CRTSCTS;
                } else {
                    tty.c_cflag |= CRTSCTS;
                }

                tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

                if (baudRateType_ == BaudRateType::STANDARD) {
                    tty.c_cflag &= ~CBAUD;
                    tty.c_cflag |= CBAUDEX;

                    switch(baudRate_) {
                        case BaudRate::B_0:
                            tty.c_ispeed = 0;
                            tty.c_ospeed = 0;
                            break;
                        case BaudRate::B_50:
                            tty.c_ispeed = 50;
                            tty.c_ospeed = 50;
                            break;
                        case BaudRate::B_75:
                            tty.c_ispeed = 75;
                            tty.c_ospeed = 75;
                            break;
                        case BaudRate::B_110:
                            tty.c_ispeed = 110;
                            tty.c_ospeed = 110;
                            break;
                        case BaudRate::B_134:
                            tty.c_ispeed = 134;
                            tty.c_ospeed = 134;
                            break;
                        case BaudRate::B_150:
                            tty.c_ispeed = 150;
                            tty.c_ospeed = 150;
                            break;
                        case BaudRate::B_200:
                            tty.c_ispeed = 200;
                            tty.c_ospeed = 200;
                            break;
                        case BaudRate::B_300:
                            tty.c_ispeed = 300;
                            tty.c_ospeed = 300;
                            break;
                        case BaudRate::B_600:
                            tty.c_ispeed = 600;
                            tty.c_ospeed = 600;
                            break;
                        case BaudRate::B_1200:
                            tty.c_ispeed = 1200;
                            tty.c_ospeed = 1200;
                            break;
                        case BaudRate::B_1800:
                            tty.c_ispeed = 1800;
                            tty.c_ospeed = 1800;
                            break;
                        case BaudRate::B_2400:
                            tty.c_ispeed = 2400;
                            tty.c_ospeed = 2400;
                            break;
                        case BaudRate::B_4800:
                            tty.c_ispeed = 4800;
                            tty.c_ospeed = 4800;
                            break;
                        case BaudRate::B_9600:
                            tty.c_ispeed = 9600;
                            tty.c_ospeed = 9600;
                            break;
                        case BaudRate::B_19200:
                            tty.c_ispeed = 19200;
                            tty.c_ospeed = 19200;
                            break;
                        case BaudRate::B_38400:
                            tty.c_ispeed = 38400;
                            tty.c_ospeed = 38400;
                            break;
                        case BaudRate::B_57600:
                            tty.c_ispeed = 57600;
                            tty.c_ospeed = 57600;
                            break;
                        case BaudRate::B_115200:
                            tty.c_ispeed = 115200;
                            tty.c_ospeed = 115200;
                            break;
                        case BaudRate::B_230400:
                            tty.c_ispeed = 230400;
                            tty.c_ospeed = 230400;
                            break;
                        case BaudRate::B_460800:
                            tty.c_ispeed = 460800;
                            tty.c_ospeed = 460800;
                            break;
                        default:
                            std::cout << "baudRate passed to " << device_ << " unrecognized." << std::endl;
                    }

                } else {
                    tty.c_cflag &= ~CBAUD;
                    tty.c_cflag |= CBAUDEX;
                    tty.c_ispeed = baudRate_;
                    tty.c_ospeed = baudRate_;
                }

                tty.c_oflag     =   0;              // No remapping, no delays
                tty.c_oflag     &=  ~OPOST;         // Make raw

                //================= CONTROL CHARACTERS (.c_cc[]) ==================//

                // c_cc[VTIME] sets the inter-character timer, in units of 0.1s.
                // Only meaningful when port is set to non-canonical mode
                // VMIN = 0, VTIME = 0: No blocking, return immediately with what is available
                // VMIN > 0, VTIME = 0: read() waits for VMIN bytes, could block indefinitely
                // VMIN = 0, VTIME > 0: Block until any amount of data is available, OR timeout occurs
                // VMIN > 0, VTIME > 0: Block until either VMIN characters have been received, or VTIME
                //                      after first character has elapsed
                // c_cc[WMIN] sets the number of characters to block (wait) for when read() is called.
                // Set to 0 if you don't want read to block. Only meaningful when port set to non-canonical mode

                if(timeout_ms_ == -1) {
                    tty.c_cc[VTIME] = 0;
                    tty.c_cc[VMIN] = 1;
                } else if(timeout_ms_ == 0) {
                    tty.c_cc[VTIME] = 0;
                    tty.c_cc[VMIN] = 0;
                } else if(timeout_ms_ > 0) {
                    tty.c_cc[VTIME] = (cc_t)(timeout_ms_/100);    // 0.5 seconds read timeout
                    tty.c_cc[VMIN] = 0;
                }

                if (softwareFlowControl_ == SoftwareFlowControl::SW_OFF) {
                    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
                } else {
                    tty.c_iflag |= (IXON | IXOFF | IXANY);
                }
                
                tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

                //=========================== LOCAL MODES (c_lflag) =======================//

                // Canonical input is when read waits for EOL or EOF characters before returning. In non-canonical mode, the rate at which
                // read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]

                tty.c_lflag	&= ~ICANON;    // Turn off canonical input, which is suitable for pass-through

                if(echo_) {
                    tty.c_lflag |= ECHO;
                } else {
                    tty.c_lflag &= ~(ECHO);
                }

                tty.c_lflag	&= ~ECHOE;     // Turn off echo erase (echo erase only relevant if canonical input is active)
                tty.c_lflag	&= ~ECHONL;    //
                tty.c_lflag	&= ~ISIG;      // Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters

                this->SetTermios2(tty);
            }

            termios2 GetTermios2() {
                struct termios2 term2;
                ioctl(fileDesc_, TCGETS2, &term2);
                return term2;
            }

            void SetTermios2(termios2 tty) {
                ioctl(fileDesc_, TCSETS2, &tty);
            }

            bool PortIsOpened() {
                bool port_open = true;
                if(state_ != State::OPEN) {
                    std::cout <<  device_ << " called but state != OPEN. Please call Open() first." << std::endl;
                    port_open = false;
                }

                if(fileDesc_ < 0) {
                    std::cout << " called but file descriptor < 0, indicating file has not been opened." << std::endl;
                    port_open = false;
                }

                return port_open;
            }

            int GetBaudRateType(int baudRate) {
                if (baudRate != BaudRateType::CUSTOM) {
                    return BaudRateType::STANDARD;
                } else {
                    return BaudRateType::CUSTOM;
                }
            }
    };
}
