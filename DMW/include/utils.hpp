#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace DMW {
namespace utils {

enum class RosTopicType {
    TOPIC,      // 普通话题: "rt/"
    REQUEST,    // 服务请求: "rq/"
    RESPONSE    // 服务响应: "rr/"
};



/**
 * @brief 对话题名称进行 ROS 兼容性修饰。
 * @param topic_name 待修饰的话题名称。
 * @return 修饰后的话题名称。
 */
inline std::string ros_topic_mangling(const std::string& topic_name, const RosTopicType& topic_type) {
    std::string topic_name_mangled = topic_name;
    if (topic_name_mangled[0] == '/') {
        topic_name_mangled = topic_name_mangled.substr(1);
    }
    if (topic_type == RosTopicType::TOPIC) {
        if (topic_name_mangled.substr(0, 3) != "rt/") {
            topic_name_mangled = "rt/" + topic_name_mangled;
        }
    }
    else if (topic_type == RosTopicType::REQUEST) {
        if (topic_name_mangled.substr(0, 3) != "rq/") {
            topic_name_mangled = "rq/" + topic_name_mangled + "Request";
        }
    } else if (topic_type == RosTopicType::RESPONSE) {
        if (topic_name_mangled.substr(0, 3) != "rr/") {
            topic_name_mangled = "rr/" + topic_name_mangled + "Reply";
        }
    } else {
        throw std::runtime_error("Invalid topic type");
    }
    return topic_name_mangled;
}

/**
 * @brief 对数据类型名称进行 ROS 兼容性修饰。
 * @param datatype_name 待修饰的数据类型名称。
 * @return 修饰后的数据类型名称。
 */
inline std::string ros_datatype_mangling(const std::string& datatype_name) {
    size_t pos = datatype_name.rfind("::");
    if (pos == std::string::npos) {
        return "dds_::" + datatype_name + "_";
    }
    std::string ns = datatype_name.substr(0, pos);
    std::string base = datatype_name.substr(pos + 2);
    return ns + "::dds_::" + base + "_";
}

/**
 * @brief 读取环境变量 DMW_ROS_COMPATIBLE。
 * @return 若环境变量值为 "1"、"true"、"TRUE" 或 "True" 则返回 true，否则返回 false。
 */
inline bool read_ros_compatible_from_env() {
    const char* env_val = std::getenv("DMW_ROS_COMPATIBLE");
    if (env_val == nullptr) {
        return false;
    }
    std::string val(env_val);
    return (val == "1" || val == "true" || val == "TRUE" || val == "True");
}

}  // namespace utils
}  // namespace DMW
