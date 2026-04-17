#include "serial_driver.hpp"
#include "logger.hpp"

#include <cstring>
#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

SerialDriver::SerialDriver():
    fd_(-1) {}

SerialDriver::~SerialDriver() {
    Close();
}

bool SerialDriver::Open(const std::string& device, SerialConfig config) {
    Close();

    int32_t baud = ParseBaudRate(config.baud);
    if (baud == -1) {
        LOG_ERROR("Invalid baud rate: {}", config.baud);
        return false;
    }

    if (config.stop_bits != 1 && config.stop_bits != 2) {
        LOG_ERROR("Invalid stop bits: {}", config.stop_bits);
        return false;
    }

    if (config.data_bits != 7 && config.data_bits != 8) {
        LOG_ERROR("Invalid data bits: {}", config.data_bits);
        return false;
    }

    if (config.parity != SerialParity::NO_PARITY && config.parity != SerialParity::EVEN_PARITY && config.parity != SerialParity::ODD_PARITY) {
        LOG_ERROR("Invalid parity mode.");
        return false;
    }

    fd_ = open(device.c_str(), config.writable ? O_RDWR : O_RDONLY);

    if (fd_ == -1) {
        LOG_ERROR("Error opening serial port: {}", device);
        return false;
    }

    struct termios term;
    if (tcgetattr(fd_, &term) < 0) {
        LOG_ERROR("Unable to get serial attributes: {}", device);
        Close();
        return false;
    }

    cfmakeraw(&term);

    if (config.stop_bits == 2) {
        term.c_cflag |= CSTOPB;
    } else {
        term.c_cflag &= ~CSTOPB;
    }

    if (config.parity == SerialParity::EVEN_PARITY) {
        term.c_cflag |= PARENB;
        term.c_cflag &= ~PARODD;
    } else if (config.parity == SerialParity::ODD_PARITY) {
        term.c_cflag |= PARENB;
        term.c_cflag |= PARODD;
    } else {
        term.c_cflag &= ~PARENB;
        term.c_cflag &= ~PARODD;
    }

    if (config.data_bits == 8) {
        term.c_cflag &= ~CSIZE;
        term.c_cflag |= CS8;
    } else {
        term.c_cflag &= ~CSIZE;
        term.c_cflag |= CS7;
    }

    if (cfsetspeed(&term, baud) < 0) {
        LOG_ERROR("Failed to set serial port baud rate: {}", config.baud);
        Close();
        return false;
    }

    if (tcsetattr(fd_, TCSAFLUSH, &term) < 0) {
        LOG_ERROR("Unable to set serial attributes: {}", device);
        Close();
        return false;
    }

    if (config.low_latency_mode && !SetLowLatencyMode()) {
        Close();
        return false;
    }

    return true;
}

void SerialDriver::Close() {
    if (fd_ < 0)
        return;

    close(fd_);
    fd_ = -1;
}

bool SerialDriver::SetLowLatencyMode() {
    if (fd_ < 0) {
        LOG_ERROR("Device not open.");
        return false;
    }

    struct serial_struct serial_info;

    if (ioctl(fd_, TIOCGSERIAL, &serial_info) < 0) {
        LOG_ERROR("Failed to set low latency mode. Cannot get serial configuration.");
        return false;
    }

    serial_info.flags |= ASYNC_LOW_LATENCY;

    if (ioctl(fd_, TIOCSSERIAL, &serial_info) < 0) {
        LOG_ERROR("Failed to set low latency mode. Cannot set serial configuration.");
        return false;
    }

    return true;
}

int32_t SerialDriver::ParseBaudRate(int32_t baud) {
    int32_t value = -1;

    if (baud == B50 || baud == 50) {
        value = B50;
    } else if (baud == B75 || baud == 75) {
        value = B75;
    } else if (baud == B110 || baud == 110) {
        value = B110;
    } else if (baud == B134 || baud == 134) {
        value = B134;
    } else if (baud == B150 || baud == 150) {
        value = B150;
    } else if (baud == B200 || baud == 200) {
        value = B200;
    } else if (baud == B300 || baud == 300) {
        value = B300;
    } else if (baud == B600 || baud == 600) {
        value = B600;
    } else if (baud == B1200 || baud == 1200) {
        value = B1200;
    } else if (baud == B1800 || baud == 1800) {
        value = B1800;
    } else if (baud == B2400 || baud == 2400) {
        value = B2400;
    } else if (baud == B4800 || baud == 4800) {
        value = B4800;
    } else if (baud == B9600 || baud == 9600) {
        value = B9600;
    } else if (baud == B19200 || baud == 19200) {
        value = B19200;
    } else if (baud == B38400 || baud == 38400) {
        value = B38400;
    } else if (baud == B57600 || baud == 57600) {
        value = B57600;
    } else if (baud == B115200 || baud == 115200) {
        value = B115200;
    } else if (baud == B230400 || baud == 230400) {
        value = B230400;
    } else if (baud == B460800 || baud == 460800) {
        value = B460800;
    } else if (baud == B576000 || baud == 576000) {
        value = B576000;
    } else if (baud == B921600 || baud == 921600) {
        value = B921600;
    } else if (baud == B1000000 || baud == 1000000) {
        value = B1000000;
    } else if (baud == B1152000 || baud == 1152000) {
        value = B1152000;
    } else if (baud == B1500000 || baud == 1500000) {
        value = B1500000;
    } else if (baud == B2000000 || baud == 2000000) {
        value = B2000000;
    } else if (baud == B2500000 || baud == 2500000) {
        value = B2500000;
    } else if (baud == B3000000 || baud == 3000000) {
        value = B3000000;
    } else if (baud == B3500000 || baud == 3500000) {
        value = B3500000;
    } else if (baud == B4000000 || baud == 4000000) {
        value = B4000000;
    }

    return value;
}

SerialResult SerialDriver::ReadBytes(std::vector<uint8_t>& output, size_t max_bytes, int32_t timeout) {
    if (fd_ < 0) {
        LOG_ERROR("Device not open.");
        return ERROR;
    }

    struct pollfd fds[1];
    fds[0].fd = fd_;
    fds[0].events = POLLIN;

    int poll_return = poll(fds, 1, timeout);
    if (poll_return == 0) {
        // AINFO_F(" Timed out while waiting for data");
        return TIMEOUT;
    } else if (poll_return < 0) {
        close(fd_);
        fd_ = -1;
        int error_num = errno;
        switch (error_num) {
            case EINTR:
                return INTERRUPTED;
            default:
                LOG_ERROR("Error polling serial port: {}", strerror(errno));
                return ERROR;
        }
    }

    size_t to_read = max_bytes;
    if (to_read <= 0) {
        int bytes;
        ioctl(fd_, FIONREAD, &bytes);
        if (bytes < 0) {
            LOG_ERROR("Error getting number of available bytes from serial port: {}", strerror(errno));
            return ERROR;
        }
        to_read = static_cast<size_t>(bytes);
    }

    size_t output_size = output.size();
    output.resize(output_size + to_read);

    int result = read(fd_, output.data() + output_size, to_read);

    if (result > 0) {
        output.resize(output_size + result);
    } else {
        output.resize(output_size);
    }

    if (result > 0) {
        return SUCCESS;
    } else if (result == 0) {
        return INTERRUPTED;
    } else {
        int error_num = errno;
        switch (error_num) {
            case EINTR:
                return INTERRUPTED;
                break;
            default:
                LOG_ERROR("Error reading serial port: {}", strerror(errno));
                return ERROR;
        }
    }
}

int32_t SerialDriver::Write(const std::vector<uint8_t>& input) {
    return write(fd_, input.data(), input.size());
}

int32_t SerialDriver::Write(const std::string& _input) {
    return write(fd_, _input.data(), _input.size());
}