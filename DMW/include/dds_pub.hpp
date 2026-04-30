#ifndef DDS_PUB_HPP
#define DDS_PUB_HPP

#include "dds_node.hpp"
#include "utils.hpp"
#include <condition_variable>
#include <cstdint>
#include "logger.hpp"

namespace DMW {
template<typename MsgPubType>
class DdsPublisher : public eprosima::fastdds::dds::DataWriterListener {
public:
    typedef typename MsgPubType::type MsgType;

    DdsPublisher(std::shared_ptr<DdsNode> node, const std::string& topic_name,
    std::atomic<bool> wait = false, const uint32_t wait_time = 0) : 
    node_(node),wait_time_(wait_time) {
        wait_.store(wait.load());
        this->type_ = eprosima::fastdds::dds::TypeSupport(new MsgPubType());
        topic_name_ = topic_name;
        type_name_ = type_.get_type_name();
        if (node_->Is_ROS_Compatible()) {
            topic_name_ = DMW::utils::ros_topic_mangling(topic_name, DMW::utils::RosTopicType::TOPIC);
            type_name_ = DMW::utils::ros_datatype_mangling(type_name_);
        }

        type_.register_type(node_->Get_Participant(), type_name_.c_str());

        //create publisher
        eprosima::fastdds::dds::PublisherQos qos = eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT;
        node_->Get_Participant()->get_default_publisher_qos(qos);
        publisher_ = node_->Get_Participant()->create_publisher(qos, nullptr, eprosima::fastdds::dds::StatusMask::none());
        if (publisher_ == nullptr) {
            throw std::runtime_error("Publisher initialization failed");
        }

        //create topic
        eprosima::fastdds::dds::TopicQos topic_qos = eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;
        node_->Get_Participant()->get_default_topic_qos(topic_qos);
        topic_ = node_->Get_Participant()->find_topic(topic_name_, eprosima::fastdds::dds::Duration_t(1, 0));
        if (topic_ == nullptr) {
            topic_ = node_->Get_Participant()->create_topic(topic_name_.c_str(), type_name_.c_str(), topic_qos, nullptr, eprosima::fastdds::dds::StatusMask::none());
        }
        if (topic_ == nullptr) {
            throw std::runtime_error("Topic initialization failed");
        }

        //create datawriter
        eprosima::fastdds::dds::DataWriterQos writer_qos = eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
        publisher_->get_default_datawriter_qos(writer_qos);
        writer_qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
        writer_qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
        writer_qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
        writer_qos.history().depth = 1;
        writer_ = publisher_->create_datawriter(topic_, writer_qos, this, eprosima::fastdds::dds::StatusMask::all());
        if (writer_ == nullptr) {
            throw std::runtime_error("Publisher writer initialization failed");
        }
    }

    ~DdsPublisher(){
        if(writer_)publisher_->delete_datawriter(writer_);
        if(publisher_) node_->Get_Participant()->delete_publisher(publisher_);
        if(topic_) node_->Get_Participant()->delete_topic(topic_);
    }

    void on_publication_matched( eprosima::fastdds::dds::DataWriter* writer, const  eprosima::fastdds::dds::PublicationMatchedStatus& info) {
        if (info.current_count_change == 1) {
            matched_ = info.current_count;
            LOG_INFO("Publisher matched.");
            cv_.notify_one();
        } else if (info.current_count_change == -1) {
            matched_ = info.current_count;
            LOG_INFO("Publisher unmatched.");
        } else {
            LOG_ERROR("{} is not a valid value for PublicationMatchedStatus current count change", info.current_count_change);
        }
    }


    bool Publish(const MsgType& message) {
        // Wait for the data endpoints discovery
        if (wait_) {
            std::unique_lock<std::mutex> matched_lock(mutex_);
            cv_.wait_for(matched_lock, std::chrono::milliseconds(wait_time_), [&]() {
                // at least one has been discovered
                return (matched_ > 0);
            });
        }
        return writer_->write(&message) == eprosima::fastdds::dds::RETCODE_OK;
    }

    /**
     * @brief get matched object number .
     */
    int32_t Get_Matched() {
        return matched_;
    }
    

private:
    eprosima::fastdds::dds::Publisher* publisher_; // Publisher
    eprosima::fastdds::dds::Topic* topic_;         // Topic
    eprosima::fastdds::dds::DataWriter* writer_;   // DataWriter
    eprosima::fastdds::dds::TypeSupport type_;     // TypeSupport for the message type
    std::string topic_name_;
    std::string type_name_;

    std::shared_ptr<DdsNode> node_;
    std::condition_variable cv_;
    std::mutex mutex_;
    int32_t matched_ = 0;

    const uint32_t wait_time_ = 5000;
    std::atomic<bool> wait_;
};
}



#endif // DDS_PUB_HPP