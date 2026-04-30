#ifndef DDS_SUB_HPP
#define DDS_SUB_HPP

#include "dds_node.hpp"
#include <condition_variable>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include "logger.hpp"

namespace DMW {

template<typename MsgSubType>
class DdsSubscription: public eprosima::fastdds::dds::DataReaderListener {
public:
    typedef typename MsgSubType::type MsgType;

    DdsSubscription(std::shared_ptr<DdsNode> node, const std::string& topic_name, const std::function<void(const MsgType&)>& callback, bool multi_pub_protect = false):
        node_(node),
        callback_(callback),
        multi_pub_protect_(multi_pub_protect) {
        this->type_ = eprosima::fastdds::dds::TypeSupport(new MsgSubType());
        this->topic_name_ = topic_name;
        this->type_name_ = this->type_.get_type_name();
        if (node_->Is_ROS_Compatible()) {
            this->topic_name_ = DMW::utils::ros_topic_mangling(topic_name, DMW::utils::RosTopicType::TOPIC);
            this->type_name_ = DMW::utils::ros_datatype_mangling(this->type_name_);
        }

        type_.register_type(node_->Get_Participant(), type_name_.c_str());

        //create subscriber
        eprosima::fastdds::dds::SubscriberQos qos = eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT;
        node_->Get_Participant()->get_default_subscriber_qos(qos);
        subscriber_ = node_->Get_Participant()->create_subscriber(qos, nullptr, eprosima::fastdds::dds::StatusMask::none());
        if (subscriber_ == nullptr) {
            throw std::runtime_error("Subscriber initialization failed");
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

        //create datareader
        eprosima::fastdds::dds::DataReaderQos reader_qos = eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
        subscriber_->get_default_datareader_qos(reader_qos);
        reader_qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::BEST_EFFORT_RELIABILITY_QOS;
        reader_qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::VOLATILE_DURABILITY_QOS;
        reader_qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_LAST_HISTORY_QOS;
        reader_qos.history().depth = 1;
        reader_ = subscriber_->create_datareader(topic_, reader_qos, this, eprosima::fastdds::dds::StatusMask::all());
        if (reader_ == nullptr) {
            throw std::runtime_error("subscriber reader initialization failed");
        }
    }

    ~DdsSubscription() {
        if (reader_ != nullptr) {
            subscriber_->delete_datareader(reader_);
        }
        if (topic_ != nullptr) {
            node_->Get_Participant()->delete_topic(topic_);
        }
        if (subscriber_ != nullptr) {
            node_->Get_Participant()->delete_subscriber(subscriber_);
        }
    }

    void on_subscription_matched(eprosima::fastdds::dds::DataReader* reader, const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) {
        if (info.current_count_change == 1) {
            matched_ = info.current_count;
            cv_.notify_one();
            LOG_INFO("Subscriber matched.");
        } else if (info.current_count_change == -1) {
            matched_ = info.current_count;
            LOG_INFO("Subscriber unmatched.");
        } else {
            LOG_ERROR("{} is not a valid value for SubscriptionMatchedStatus current count change", info.current_count_change);
        }
    }

    void on_data_available(eprosima::fastdds::dds::DataReader* reader) {
        while (eprosima::fastdds::dds::RETCODE_OK == reader->take_next_sample(&current_message_, &current_message_info_)) {
            if ((current_message_info_.instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE) && current_message_info_.valid_data) {
                if (callback_) {
                    if (this->multi_pub_protect_) {
                        if (matched_ > 1) {
                            LOG_INFO("Detect multiple publisher publish same topic message.");
                        } else {
                            callback_(current_message_);
                        }
                    } else {
                        callback_(current_message_);
                    }
                }
            }
        }
    }

    int32_t Get_Matched() {
        return matched_;
    }

private:
    eprosima::fastdds::dds::Subscriber* subscriber_; // Subscriber
    eprosima::fastdds::dds::Topic* topic_;           // Topic
    eprosima::fastdds::dds::DataReader* reader_;     // DataReader
    eprosima::fastdds::dds::TypeSupport type_;       // TypeSupport for the message type
    std::string topic_name_;
    std::string type_name_;

    MsgType current_message_;
    eprosima::fastdds::dds::SampleInfo current_message_info_;
    bool matched_ = 0;

    std::condition_variable cv_;
    std::mutex mutex_;

    std::shared_ptr<DdsNode> node_;
    std::function<void(const MsgType&)> callback_;
    bool multi_pub_protect_;
};

} // namespace DMW
#endif // DDS_SUB_HPP