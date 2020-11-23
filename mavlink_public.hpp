#ifndef MAVLINK_PUBLIC_HPP
#define MAVLINK_PUBLIC_HPP
#include <common/mavlink.h>
bool decode_mavlink_message(mavlink_message_t &msg)
{
//    mutex.lock();
    switch(msg.msgid)
    {
         case MAVLINK_MSG_ID_HEARTBEAT:
         {
             mavlink_heartbeat_t heartbeat;
             mavlink_msg_heartbeat_decode(&msg, &heartbeat);
             std::cout << "heartbeat" << std::endl;
             // access message specific fields
              std::cout << "    type:            "            << (uint)heartbeat.type << std::endl;
//              std::cout << "    autopilot:       "       << (uint)heartbeat.autopilot << std::endl;
//              std::cout << "    base_mode:       "       << (uint)heartbeat.base_mode << std::endl;
//              std::cout << "    custom_mode:     "     << (uint)heartbeat.custom_mode << std::endl;
              std::cout << "    system_status:   "   << (uint)heartbeat.system_status << std::endl;
//              std::cout << "    mavlink_version: " << (uint)heartbeat.mavlink_version << std::endl;
             break;
         }
         case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
         {
             mavlink_param_request_list_t param_request_list;
             mavlink_msg_param_request_list_decode(&msg, &param_request_list);
             std::cout << "param request list" << std::endl;
             break;
         }
         case MAVLINK_MSG_ID_ATTITUDE_QUATERNION :
         {
             mavlink_attitude_quaternion_t attitude_quaternion;
             mavlink_msg_attitude_quaternion_decode(&msg, &attitude_quaternion);
             std::cout << "attitude_quaternion" << "w:" << " " 
                       << (float)attitude_quaternion.q1 << " "
                       << "x:" << (float)attitude_quaternion.q2 << " "
                       << "y:" << (float)attitude_quaternion.q3 << " "
                       << "z:" << (float)attitude_quaternion.q4 << " "
                       <<std::endl;
             break;
         }
         case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
         {
             mavlink_local_position_ned_t local_position_ned;
             mavlink_msg_local_position_ned_decode(&msg, &local_position_ned);
             std::cout << "local position ned" << std::endl;
             break;
         }
         case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
         {
             mavlink_mission_set_current_t mission_set_current;
             mavlink_msg_mission_set_current_decode(&msg, &mission_set_current);
             std::cout << "mission set current" << std::endl;
             break;
         }
         case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
         {
             mavlink_request_data_stream_t request_data_stream;
             mavlink_msg_request_data_stream_decode(&msg, &request_data_stream);
             std::cout << "request data stream" << std::endl;
             break;
         }
         case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
         {
             mavlink_named_value_float_t named_value_float;
             mavlink_msg_named_value_float_decode(&msg, &named_value_float);
             std::cout << "named value float" << std::endl;
             break;
         }
         case MAVLINK_MSG_ID_STATUSTEXT:
         {
             mavlink_statustext_t statustext;
             mavlink_msg_statustext_decode(&msg, &statustext);
             std::cout << "status text" << std::endl;
             std::cout << statustext.text << std::endl;
             break;
         }
        case MAVLINK_MSG_ID_HOME_POSITION:
            std::cout << "HOME_POSITION" << std::endl;
            break;
        case MAVLINK_MSG_ID_RADIO_STATUS:
            std::cout << "RADIO_STATUS" << std::endl;
            break;
        case MAVLINK_MSG_ID_RC_CHANNELS:
            std::cout << "RC_CHANNELS" << std::endl;
            break;
        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
            std::cout << "RC_CHANNELS_RAW" << std::endl;
            break;
        case MAVLINK_MSG_ID_BATTERY_STATUS:
            std::cout << "BATTERY_STATUS" << std::endl;
            break;
        case MAVLINK_MSG_ID_SYS_STATUS:
            std::cout << "SYS_STATUS" << std::endl;
            break;
        case MAVLINK_MSG_ID_RAW_IMU:
            std::cout << "RAW_IMU" << std::endl;
            break;
        case MAVLINK_MSG_ID_SCALED_IMU:
            std::cout << "SCALED_IMU" << std::endl;
            break;
        case MAVLINK_MSG_ID_SCALED_IMU2:
            std::cout << "SCALED_IMU2" << std::endl;
            break;
        case MAVLINK_MSG_ID_COMMAND_ACK:
            std::cout << "COMMAND_ACK" << std::endl;
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
            std::cout << "COMMAND_LONG" << std::endl;
            break;
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
            std::cout << "AUTOPILOT_VERSION" << std::endl;
            break;
        case MAVLINK_MSG_ID_WIND_COV:
            std::cout << "WIND_COV" << std::endl;
            break;
        case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
            std::cout << "ACTUATOR_CONTROLS" << std::endl;
            break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:
            std::cout << "GPS_RAW_INT" << std::endl;
            break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            std::cout << "GLOBAL_POSITION_INT" << std::endl;
            break;
        case MAVLINK_MSG_ID_ALTITUDE:
            std::cout << "MSG_ID_ALTITUDE" << std::endl;
            break;
        case MAVLINK_MSG_ID_VFR_HUD:
            std::cout << "VFR_HUD" << std::endl;
            break;
        case MAVLINK_MSG_ID_SCALED_PRESSURE:
            std::cout << "CALED_PRESSURE" << std::endl;
            break;
        case MAVLINK_MSG_ID_SCALED_PRESSURE2:
            std::cout << "CALED_PRESSURE2" << std::endl;
            break;    
        case MAVLINK_MSG_ID_ATTITUDE:
            std::cout << "ATTITUDE" << std::endl;
            break;
        case MAVLINK_MSG_ID_ATTITUDE_TARGET:
            std::cout << "ATTITUDE_TARGET" << std::endl;
            break;
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
            std::cout << "DISTANCE_SENSOR" << std::endl;
            break;
        case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
            std::cout << "ESTIMATOR_STATUS" << std::endl;
            break;
#ifdef mavlink_v2
        case MAVLINK_MSG_ID_PROTOCOL_VERSION:
            std::cout << "PROTOCOL_VERSION" << std::endl;
            break;
        
        case MAVLINK_MSG_ID_MOUNT_ORIENTATION:
            std::cout << "MOUNT_ORIENTATION" << std::endl;
            break;
        case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
            std::cout << "OBSTACLE_DISTANCE" << std::endl;
            break;
        case MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED:
            std::cout << "CAMERA_IMAGE_CAPTURED" << std::endl;
            break;
        case MAVLINK_MSG_ID_LOGGING_DATA:
            std::cout << "LOGGING_DATA" << std::endl;
            break;
        case MAVLINK_MSG_ID_LOGGING_DATA_ACKED:
            std::cout << "LOGGING_DATA_ACKED" << std::endl;
            break;
#endif            
        default:
        {
            // std::cout << "Unsupported packet -> ";
            std::cout << "SYS: "     << (int)msg.sysid;
            std::cout << ", COMP: "   << (int)msg.compid;
            std::cout << ", SEQ: "    << (int)msg.seq;
            std::cout << ", LEN: "    << (int)msg.len;
            std::cout << ", MSG ID: " << (int)msg.msgid << std::endl; 
//            std::cout << ", MSG payload: " ;
//            for(int i = 0 ; i < (int)msg.len; i++ )
//                std::cout << (int)msg.payload64[i] << " ";           
//            std::cout << std::endl;
            break;
        }
    }
    std::cout << "----------------------" << std::endl;
//    mutex.unlock();

    return true;
}
#endif // MAVLINK_PUBLIC_HPP
