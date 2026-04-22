#ifndef DDS_NODE_HPP
#define DDS_NODE_HPP
#include "utils.hpp"

#include "utils.hpp"
#include <cstdlib>
#include <fastdds/LibrarySettings.hpp>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>

#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>

#include <fastdds/dds/topic/TypeSupport.hpp>

#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>

#include <fastdds/rtps/attributes/BuiltinTransports.hpp>
#include <fastdds/rtps/common/GuidPrefix_t.hpp>
#include <fastdds/rtps/common/WriteParams.hpp>
#include <fastdds/rtps/participant/ParticipantDiscoveryInfo.hpp>
#include <fastdds/rtps/transport/TCPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.hpp>

namespace DMW {

class DdsNode {
public:
    /**
    * @brief 构造函数
    * @param domain_id DDS 域 ID
    * @param mode 0: 默认模式；1: 共享内存模式；2: 大数据模式；3: 混合模式（默认使用共享内存传输，自动fallback到其他传输）
    * @param ros_compatible 可选参数，用户自定义模式下指定是否启用 ROS 兼容性（默认为 false）
    */
    explicit DdsNode(uint32_t domain_id, uint8_t mode = 0, std::optional<bool> ros_compatible = std::nullopt) {
        eprosima::fastdds::dds::DomainParticipantQos qos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
        eprosima::fastdds::LibrarySettings library_settings;
        library_settings.intraprocess_delivery = eprosima::fastdds::IntraprocessDeliveryType::INTRAPROCESS_OFF;

        if (mode == 0) {
            //0: 默认模式
            qos.transport().use_builtin_transports = true;
        } else if (mode == 1) {
            //1: 共享内存模式
            qos.transport().use_builtin_transports = false;
            std::shared_ptr<eprosima::fastdds::rtps::SharedMemTransportDescriptor> shm_transport_ =
                std::make_shared<eprosima::fastdds::rtps::SharedMemTransportDescriptor>();
            shm_transport_->segment_size(shm_transport_->max_message_size() * 10);
            qos.transport().user_transports.push_back(shm_transport_);
        } else if (mode == 2) {
            //2: 大数据模式
            qos.transport().use_builtin_transports = true;
            qos.setup_transports(eprosima::fastdds::rtps::BuiltinTransports::LARGE_DATA);
        } else if (mode == 3) {
            // 3: 混合模式
            qos.transport().use_builtin_transports = false;

            // 添加LARGE_DATA的TCP传输（用于跨主机）
            auto tcp_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
            tcp_transport->sendBufferSize = 16 * 1024 * 1024;
            tcp_transport->receiveBufferSize = 16 * 1024 * 1024;
            qos.transport().user_transports.push_back(tcp_transport);

            // 添加UDP传输（用于发现和小消息）
            auto udp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
            qos.transport().user_transports.push_back(udp_transport);

            // 添加大容量SHM（本机优先使用，零拷贝）
            auto shm_transport = std::make_shared<eprosima::fastdds::rtps::SharedMemTransportDescriptor>();
            shm_transport->segment_size(16 * 1024 * 1024);    // 16MB段大小
            shm_transport->max_message_size(8 * 1024 * 1024); // 8MB最大消息
            qos.transport().user_transports.push_back(shm_transport);
        } else {
            qos.transport().use_builtin_transports = true;
        }

        auto factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();
        factory->set_library_settings(library_settings);

        participant_ = factory->create_participant(domain_id, qos, nullptr, eprosima::fastdds::dds::StatusMask::none());
        if (participant_ == nullptr) {
            throw std::runtime_error("Participant initialization failed");
        }

        // Determine is_ros_compatible_ value: parameter > environment variable > false
        if (ros_compatible.has_value()) {
            ros_compatible_ = ros_compatible.value();
        } else {
            ros_compatible_ = DMW::utils::read_ros_compatible_from_env();
        }
    }

    /**
    * @brief 析构函数，负责清理 DDS 参与者资源
    */
    ~DdsNode() {
        if (participant_ != nullptr) {
            eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
        }
    }

    eprosima::fastdds::dds::DomainParticipant* Get_Participant() const {
        return participant_;
    }

    bool Is_ROS_Compatible() const {
        return ros_compatible_;
    }

private:
    eprosima::fastdds::dds::DomainParticipant* participant_;
    std::string namespace_;
    bool ros_compatible_;
};

} // namespace DMW

#endif // DDS_NODE_HPP