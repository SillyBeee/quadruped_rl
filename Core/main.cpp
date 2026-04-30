#include <iostream>
#include <string>
#include <vector>

#include "logger.hpp"
#include "serial_driver.hpp"
#include "dclcpp.hpp"  
#include "torquePubSubTypes.hpp"
#include "functional"
#include <csignal>
#include "thread"

int main() {
    signal(SIGINT, [](int signum) {
        LOG_INFO("Interrupt signal ({}) received. Exiting...", signum);
        std::exit(signum);
    });
    
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
    std::shared_ptr<DMW::DdsNode> node = std::make_shared<DMW::DdsNode>(111,0);

    auto callback = [](const Torque& msg) {
        LOG_INFO("Received message: torque = {}", msg.torque());
    };

    
    DMW::DdsSubscription<TorquePubSubType> sub (node,"/test_topic",std::bind(callback, std::placeholders::_1), true);

    DMW::DdsPublisher<TorquePubSubType> pub(node, "/test_topic");


    while(1){
        Torque msg;
        msg.torque(123.45);
        pub.Publish(msg);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }




    LOG_INFO("DMW header included successfully");

    return 0;
}