/**
 * @file dds_base.hpp
 * @author SillyBee
 * @brief DDS Middle Ware data base class
 * @version 0.1
 * @details 
 * @date 2026-4-17
 * @update 2026-4-17
 * @copyleft  Copyleft (c) 2026 
 */


#include <chrono>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

