#ifndef SERIAL_SENDER_H
#define SERIAL_SENDER_H

#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>

class SerialSender {
public:
    SerialSender(const std::string &port) {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ == -1) {
            std::cerr << "Failed to open serial port." << std::endl;
            return;
        }
        configurePort();
    }

    ~SerialSender() {
        close(fd_);
    }
    
    bool sendData(const std::string &data) {
        if (fd_ != -1) {
            write(fd_, data.c_str(), data.size());
            std::string buffer;
            return this->readUntil(buffer);
        }
        return false;
    }

    void ReadData(std::string &buffer, size_t size) {
        if (fd_ != -1) {
            char temp_buffer[256];
            ssize_t bytes_read = read(fd_, temp_buffer, size);
            if (bytes_read > 0) {
                buffer.assign(temp_buffer, bytes_read);
            }
        }
    }
     bool readUntil(std::string &buffer, char delimiter = '\n', int timeout_ms = 1000) {
        if (fd_ == -1) return false;
        
        buffer.clear();
        int time_out_count=0;
        while (true) {            
            // 读取一个字符
            char ch;
            ssize_t bytes_read = read(fd_, &ch, 1);
            
            if (bytes_read > 0) {
                buffer += ch;
                if (ch == delimiter) {
                    return true; // 找到结束符
                }
            } else if (bytes_read == 0) {
                // 没有数据可读，短暂等待后重试
                usleep(10000); // 10ms
                time_out_count++;
                if(time_out_count>timeout_ms/10)
                {
                    return false;
                }
            } else {
                // 读取错误
                return false;
            }
        }
    }

private:
    int fd_;

    void configurePort() {
        struct termios options;
        tcgetattr(fd_, &options);
        cfsetispeed(&options, B115200); // Set baud rate to 115200
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
        options.c_cflag &= ~PARENB; // No parity
        options.c_cflag &= ~CSTOPB; // 1 stop bit
        options.c_cflag &= ~CSIZE; // Clear current char size mask
        options.c_cflag |= CS8; // 8 data bits
        tcsetattr(fd_, TCSANOW, &options);
    }
};

#endif // SERIAL_SENDER_H