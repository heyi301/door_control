#ifndef SERIAL_SENDER_H
#define SERIAL_SENDER_H

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <thread>
#include <sys/stat.h>

class SerialSender {
public:
    SerialSender(const std::string &port, int baud = B115200)
    : fd_(-1), port_(port), baud_(baud),
      last_reopen_(std::chrono::steady_clock::time_point::min()),
      reopen_backoff_ms_(2000)
    {
        if (!reopen()) {
            std::cerr << "SerialSender: initial open failed for " << port_ << std::endl;
        }
    }

    ~SerialSender() {
        if (fd_ != -1) close(fd_);
    }

    // 发送 data 并等待以 delimiter 结尾的应答（默认 '#'，可按需修改）
    // 返回 true 表示成功读到以 delimiter 结尾的应答
    bool sendData(const std::string &data, char delimiter = '#', int timeout_ms = 1000) {
        if (!ensureOpen()) {
            std::cerr << "sendData: port not available: " << port_ << std::endl;
            return false;
        }

        // flush input before sending to avoid reading stale bytes
        tcflush(fd_, TCIFLUSH);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        const char *ptr = data.c_str();
        size_t remaining = data.size();
        int reopen_attempts = 0;

        auto write_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

        while (remaining > 0) {
            if (std::chrono::steady_clock::now() >= write_deadline) {
                std::cerr << "sendData: write timeout after " << timeout_ms << " ms" << std::endl;
                return false;
            }

            ssize_t written = write(fd_, ptr, remaining);
            if (written > 0) {
                ptr += written;
                remaining -= written;
                continue;
            }

            if (written < 0) {
                if (errno == EINTR) {
                    continue;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // wait a bit for fd to be writable (bounded)
                    fd_set wf;
                    FD_ZERO(&wf);
                    FD_SET(fd_, &wf);
                    struct timeval tv;
                    tv.tv_sec = 0;
                    tv.tv_usec = 100 * 1000; // 100 ms
                    int sel = select(fd_ + 1, nullptr, &wf, nullptr, &tv);
                    if (sel > 0) continue;
                    continue;
                }
                if (errno == EIO) {
                    std::cerr << "write EIO on " << port_ << ": " << strerror(errno) << " -- attempting reopen" << std::endl;
                    if (++reopen_attempts <= 3 && reopen()) {
                        continue;
                    }
                    std::cerr << "write failed after reopen attempts: " << strerror(errno) << std::endl;
                    return false;
                }
                std::cerr << "write failed: " << strerror(errno) << std::endl;
                return false;
            }

            // written == 0 -> unexpected, small sleep and retry
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // wait for output queue to drain, but do not block indefinitely
        if (!waitOutputDrain(500)) {
            std::cerr << "waitOutputDrain timed out (port=" << port_ << ")" << std::endl;
            // continue to read anyway
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        std::string buffer;
        bool ok = readUntil(buffer, delimiter, timeout_ms);
        if (!ok) {
            std::cerr << "readUntil timeout or error (port=" << port_ << ", data=\"" << data << "\")" << std::endl;
        } else {
            std::cout << "Received response: " << buffer << std::endl;
        }
        return ok;
    }

    // 直接读取任意可用数据，最多 size 字节
    void ReadData(std::string &buffer, size_t size) {
        buffer.clear();
        if (fd_ == -1) return;
        char temp_buffer[512];
        ssize_t bytes_read = read(fd_, temp_buffer, std::min<size_t>(size, sizeof(temp_buffer)));
        if (bytes_read > 0) buffer.assign(temp_buffer, bytes_read);
        else if (bytes_read < 0) std::cerr << "ReadData read error: " << strerror(errno) << std::endl;
    }

    // 读取直到遇到 delimiter 或超时（ms）
    bool readUntil(std::string &buffer, char delimiter = '#', int timeout_ms = 1000) {
        buffer.clear();
        if (fd_ == -1) return false;

        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        const int poll_ms = 50;

        while (std::chrono::steady_clock::now() < deadline) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(fd_, &readfds);
            struct timeval tv;
            tv.tv_sec = poll_ms / 1000;
            tv.tv_usec = (poll_ms % 1000) * 1000;

            int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
            if (ret < 0) {
                if (errno == EINTR) continue;
                std::cerr << "select error: " << strerror(errno) << std::endl;
                return false;
            } else if (ret == 0) {
                continue;
            }

            if (FD_ISSET(fd_, &readfds)) {
                char buf[128];
                ssize_t n = read(fd_, buf, sizeof(buf));
                if (n > 0) {
                    buffer.append(buf, buf + n);
                    auto pos = buffer.find(delimiter);
                    if (pos != std::string::npos) {
                        buffer = buffer.substr(0, pos + 1);
                        return true;
                    }
                } else if (n == 0) {
                    // EOF from device -> treat as error to avoid hanging
                    std::cerr << "readUntil: device EOF (n==0)" << std::endl;
                    return false;
                } else {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        continue;
                    }
                    std::cerr << "read error: " << strerror(errno) << std::endl;
                    return false;
                }
            }
        }
        return false;
    }

private:
    int fd_;
    std::string port_;
    int baud_;
    std::chrono::steady_clock::time_point last_reopen_;
    int reopen_backoff_ms_;

    bool fileExists() const {
        struct stat st;
        return (stat(port_.c_str(), &st) == 0);
    }

    // only attempt reopen if enough time elapsed since last attempt
    bool ensureOpen() {
        if (fd_ != -1) return true;
        auto now = std::chrono::steady_clock::now();
        if (!fileExists()) {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_reopen_).count() < reopen_backoff_ms_) {
                return false;
            }
            last_reopen_ = now;
            return reopen();
        }
        // if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_reopen_).count() < 200) {
        //     return false;
        // }
        last_reopen_ = now;
        return reopen();
    }

    bool reopen() {
        if (fd_ != -1) {
            close(fd_);
            fd_ = -1;
        }

        if (!fileExists()) {
            std::cerr << "Failed to open serial port " << port_ << ": device node not found" << std::endl;
            return false;
        }

        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ == -1) {
            std::cerr << "Failed to open serial port " << port_ << ": " << strerror(errno) << std::endl;
            return false;
        }

        int flags = fcntl(fd_, F_GETFL, 0);
        if (flags != -1) fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK & ~O_NDELAY);

        configurePort();
        tcflush(fd_, TCIOFLUSH);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return true;
    }

    void configurePort() {
        struct termios options;
        if (tcgetattr(fd_, &options) != 0) {
            std::cerr << "tcgetattr failed: " << strerror(errno) << std::endl;
            return;
        }

        cfmakeraw(&options);
        cfsetispeed(&options, baud_);
        cfsetospeed(&options, baud_);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 1; // 0.1s
        if (tcsetattr(fd_, TCSANOW, &options) != 0) {
            std::cerr << "tcsetattr failed: " << strerror(errno) << std::endl;
        }
    }

    // non-blocking check for pending output bytes; wait up to timeout_ms for queue to empty
    bool waitOutputDrain(int timeout_ms) {
        if (fd_ == -1) return false;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            int pending = 0;
            if (ioctl(fd_, TIOCOUTQ, &pending) == -1) {
                // ioctl failed — bail out
                return false;
            }
            if (pending == 0) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }
};

#endif // SERIAL_SENDER_H