#include <iostream>
#include <string>
#include <vector>

#include "logger.hpp"
#include "serial_driver.hpp"
#include "dcl.hpp"  

int main() {
    // 1) Logger
    logger::Logger::GetInstance().Init(
        logger::LogLevel::INFO,
        logger::LogLevel::DEBUG
    );
    LOG_INFO("Core app start");

    // 2) Serial Driver
    SerialDriver serial;
    SerialConfig cfg;
    cfg.baud = 115200;
    cfg.data_bits = 8;
    cfg.stop_bits = 1;
    cfg.parity = NO_PARITY;
    cfg.writable = true;

    const std::string dev = "/dev/ttyUSB0";
    if (!serial.Open(dev, cfg)) {
        LOG_WARN("Open serial failed: {}", dev);
    } else {
        std::vector<uint8_t> out{0x55, 0xAA, 0x01, 0x02};
        const int32_t n = serial.Write(out);
        LOG_INFO("Write {} bytes to {}", n, dev);
        serial.Close();
    }

    // 3) DMW
    // 当前 DMW 仅有头文件包含，尚无可实例化接口。
    // 这里先证明 Core 已接入 DMW 头文件依赖。
    LOG_INFO("DMW header included successfully");

    return 0;
}