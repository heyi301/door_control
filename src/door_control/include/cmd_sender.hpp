#ifndef SERIAL_SENDER_H
#define SERIAL_SENDER_H

#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <cstring>

class SerialSender {
public:
    SerialSender(const std::string &port) : fd_(-1) {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ == -1) {
            std::cerr << "Failed to open serial port " << port << ": " << strerror(errno) << std::endl;
            return;
        }

        // 清除非阻塞标志（如果之前使用了 O_NDELAY / O_NONBLOCK）
        int flags = fcntl(fd_, F_GETFL, 0);
        if (flags != -1) {
            fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK & ~O_NDELAY);
        }

        configurePort();

        // 清空输入输出缓冲区
        tcflush(fd_, TCIOFLUSH);
    }

    ~SerialSender() {
        if (fd_ != -1) close(fd_);
    }
    
    // 发送 data 并等待以 delimiter 结尾的应答（默认 '#'，可按需修改）
    bool sendData(const std::string &data, char delimiter = '#', int timeout_ms = 1000) {
        if (fd_ == -1) {
            std::cerr << "sendData: invalid fd" << std::endl;
            return false;
        }

        ssize_t written = write(fd_, data.c_str(), data.size());
        if (written < 0) {
            std::cerr << "write failed: " << strerror(errno) << std::endl;
            return false;
        }
        // 等待并读取应答
        std::string buffer;
        bool ok = this->readUntil(buffer, delimiter, timeout_ms);
        if (!ok) {
            std::cerr << "readUntil timeout or error" << std::endl;
        } else {
            std::cout << "Received response: " << buffer << std::endl;
        }
        return ok;
    }

    void ReadData(std::string &buffer, size_t size) {
        buffer.clear();
        if (fd_ != -1) {
            char temp_buffer[256];
            ssize_t bytes_read = read(fd_, temp_buffer, std::min(size, sizeof(temp_buffer)));
            if (bytes_read > 0) {
                buffer.assign(temp_buffer, bytes_read);
            } else if (bytes_read < 0) {
                std::cerr << "ReadData read error: " << strerror(errno) << std::endl;
            }
        }
    }

    // 读取直到遇到 delimiter 或超时（使用 select 进行可读等待）
    bool readUntil(std::string &buffer, char delimiter = '#', int timeout_ms = 1000) {
        if (fd_ == -1) return false;
        
        buffer.clear();
        int elapsed_ms = 0;
        const int step_ms = 50; // 每次 select 的等待步长
        while (elapsed_ms < timeout_ms) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(fd_, &readfds);
            struct timeval tv;
            tv.tv_sec = step_ms / 1000;
            tv.tv_usec = (step_ms % 1000) * 1000;

            int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
            if (ret < 0) {
                if (errno == EINTR) continue;
                std::cerr << "select error: " << strerror(errno) << std::endl;
                return false;
            } else if (ret == 0) {
                // 超时片段，继续循环直到总体 timeout_ms
                elapsed_ms += step_ms;
                continue;
            }

            if (FD_ISSET(fd_, &readfds)) {
                char ch;
                ssize_t n = read(fd_, &ch, 1);
                if (n > 0) {
                    buffer += ch;
                    if (ch == delimiter) {
                        return true;
                    }
                } else if (n == 0) {
                    // 设备已关闭或没有数据
                    elapsed_ms += step_ms;
                } else {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        elapsed_ms += step_ms;
                        continue;
                    }
                    std::cerr << "read error: " << strerror(errno) << std::endl;
                    return false;
                }
            }
        }
        return false; // 超时
    }

private:
    int fd_;

    void configurePort() {
        struct termios options;
        if (tcgetattr(fd_, &options) != 0) {
            std::cerr << "tcgetattr failed: " << strerror(errno) << std::endl;
            return;
        }

        // 设置原始模式（raw），关闭回显、行缓冲等
        cfmakeraw(&options);

        // 波特率 115200
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        // 非规范模式下的读取行为：
        // VMIN = 0, VTIME = 1 -> read 最多等待 0.1s（1 * 0.1s），然后返回已读数据
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 1;

        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            std::cerr << "tcsetattr failed: " << strerror(errno) << std::endl;
        }
    }
};

#endif // SERIAL_SENDER_H