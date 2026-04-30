#include <iostream>
#include <string>
#include <vector>

#include "dds_node.hpp"
#include "dds_sub.hpp"
#include "logger.hpp"
#include "serial_driver.hpp"
#include "dclcpp.hpp"  
#include "torquePubSubTypes.hpp"

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
    DMW::DdsNode node (111,0);

    
    // DMW::DdsSubscription<TorquePubSubType> sub (node,"/test_topic",);


    LOG_INFO("DMW header included successfully");

    return 0;
}