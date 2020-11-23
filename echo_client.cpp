/*
 * Copyright (c) 2016, Peter Thorson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the WebSocket++ Project nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL PETER THORSON BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/memory.hpp>
#include "strconvert.hpp"
#include "mavlink_public.hpp"
#include <chrono>
typedef websocketpp::client<websocketpp::config::asio_client> client;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;
using websocketpp::lib::thread;

mavlink_status_t status;
// pull out the type of messages sent by our config
typedef websocketpp::config::asio_client::message_type::ptr message_ptr;
websocketpp::connection_hdl hdl_;
auto start = std::chrono::system_clock::now();
auto end = std::chrono::system_clock::now();
// This message handler will be invoked once for each incoming message. It
// prints the message and then sends a copy of the message back to the server.
void trim(std::string &s)
  {
     
//     if( !s.empty() )
//     {
//         s.erase(0,s.find_first_not_of(" "));
//         s.erase(s.find_last_not_of(" ") + 1);
//     }
     
     int index = 0;
     if( !s.empty())
     {
         while( (index = s.find(' ',index)) != std::string::npos)
         {
             s.erase(index,1);
         }
     }
 
 }
int get_mavmsg(std::string recvstr, mavlink_message_t &mav_msg)
{
#ifdef mavlink_v2
    if(recvstr[0] != 'F'  || recvstr[1] != 'D' || recvstr.size() < 5){
        return -1;
    }
#elif  mavlink_v1 
    if(recvstr[0] != 'F'  || recvstr[1] != 'E' || recvstr.size() < 5){
        return -1;
    }
#endif
    else{
//        std::cout <<"[mavlink to string:]"<< recvstr << std::endl;
        if(recvstr.size() < 10) return 0;
    //    std::cout <<"before trim:"<< recvstr << " length " << recvstr.size()<< std::endl; 
        trim(recvstr);
//        std::cout <<"[after trim, length:"<< recvstr.size() << "] "<< recvstr  << std::endl;
        char ptr[recvstr.size()];
        strcpy(ptr,recvstr.data());
//        std::cout <<"[copied char array]"<<"[len:"<< strlen(ptr) <<"]"<< ptr << std::endl;       
        unsigned char ubuffer[strlen(ptr)/2];
        convertStrToUnChar(ptr, ubuffer);
        int packlen = sizeof(ubuffer)/sizeof(ubuffer[0]);
//        std::cout << "[ubuffer length:"<< packlen << "]"<<std::endl;

        for(int i = 0;i < packlen; i++)
        {
            if(mavlink_parse_char(MAVLINK_COMM_0, ubuffer[i], &mav_msg, &status))
            {
                decode_mavlink_message(mav_msg);
                return 1;
            }
        }
        
        return 0;
    }
}
void on_message(client* c, websocketpp::connection_hdl hdl, message_ptr msg) {
    hdl_ = hdl;
    if(msg->get_opcode() == websocketpp::frame::opcode::text){
        std::cout << "on_message called with hdl: " << hdl.lock().get()
                  << " and message: " << msg->get_payload()
                  << std::endl;
    } else {
        std::cout << "on_message called with hdl: " << hdl.lock().get()
                  << " and message: " << websocketpp::utility::to_hex(msg->get_payload())
                  << std::endl;
    }
    std::cout << "------------------------------------------------------------" <<std::endl;
    
    mavlink_message_t mav_msg;
    std::string recvstr = websocketpp::utility::to_hex(msg->get_payload());
    get_mavmsg(recvstr, mav_msg);
    
//    mavlink_msg_heartbeat_pack(2, 1, &mav_msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_STANDBY);

//    uint8_t buf[512];
//    uint16_t len;
//    len = mavlink_msg_to_send_buffer(buf, &mav_msg);
//    websocketpp::lib::error_code ec;
//    c->send(hdl, buf, len, websocketpp::frame::opcode::binary, ec);

//    if (ec) {
//        std::cout << "Echo failed because: " << ec.message() << std::endl;
//    }
    /*
    mavlink_command_long_t mav_cmd;
    mav_cmd.command = 22;
    mav_cmd.param3 = 0;
    mav_cmd.param7 = 2.0;
    mavlink_msg_command_long_encode(0, 1, &mav_msg, &mav_cmd);
    */
}
void respondWithMavlinkMessage(client* c, const mavlink_message_t& msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    websocketpp::lib::error_code ec;
    c->send(hdl_, buffer, len, websocketpp::frame::opcode::binary, ec);
    if (ec) {
        std::cout << "Echo failed because: " << ec.message() << std::endl;
    }
}

int get_boot_time(){
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    return duration.count();
}
void heart_messages(client* c){

    mavlink_message_t msg;
    uint8_t _vehicleSystemId = '1' - '0';
    uint8_t _vehicleComponentId = '1' - '0';
    uint8_t _mavlinkChannel = '0' - '0';
    int count = 0;
    float roll = 0.0, pitch = 0.0, yaw = 0.0;
    float roll_speed = 0.08, pitch_speed = 0.08, yaw_speed = 0.08;
    float local_ned_x = 0, local_ned_y = 0, local_ned_z = 0;
    float ned_vx = 0, ned_vy = 0, ned_vz = 0;
    while(1){
        //---------mavlink heartbeat---------------//
        mavlink_msg_heartbeat_pack(_vehicleSystemId, _vehicleComponentId, &msg, MAV_TYPE_GENERIC,
                                   MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_STANDBY);
        respondWithMavlinkMessage(c, msg);
        //-------------home position---------------//
        float bogus[4];
        bogus[0] = 0.0f;
        bogus[1] = 0.0f;
        bogus[2] = 0.0f;
        bogus[3] = 0.0f;
        double _vehicleLatitude = 107.40;
        double _vehicleLongitude = 33.42;
        double _vehicleAltitude = 1.5;
#ifdef mavlink_v2
        mavlink_msg_home_position_pack_chan(
                _vehicleSystemId,
                _vehicleComponentId,
                _mavlinkChannel,
                &msg,
                (int32_t)(_vehicleLatitude * 1E7),
                (int32_t)(_vehicleLongitude * 1E7),
                (int32_t)(_vehicleAltitude * 1000),
                0.0f, 0.0f, 0.0f,
                &bogus[0],
                0.0f, 0.0f, 0.0f,
                0);
#elif mavlink_v1
        mavlink_msg_home_position_pack_chan(
                _vehicleSystemId,
                _vehicleComponentId,
                _mavlinkChannel,
                &msg,
                (int32_t)(_vehicleLatitude * 1E7),
                (int32_t)(_vehicleLongitude * 1E7),
                (int32_t)(_vehicleAltitude * 1000),
                0.0f, 0.0f, 0.0f,
                &bogus[0],
                0.0f, 0.0f, 0.0f);
#endif
        respondWithMavlinkMessage(c, msg);
        //-------------SysStatus---------------//
        int8_t _batteryRemaining = static_cast<int8_t>(100 - 50);
        mavlink_msg_sys_status_pack_chan(
                _vehicleSystemId,
                _vehicleComponentId,
                static_cast<uint8_t>(_mavlinkChannel),
                &msg,
                0,          // onboard_control_sensors_present
                0,          // onboard_control_sensors_enabled
                0,          // onboard_control_sensors_health
                250,        // load
                4200 * 4,   // voltage_battery
                8000,       // current_battery
                _batteryRemaining, // battery_remaining
                0,0,0,0,0,0);
        respondWithMavlinkMessage(c, msg);
        roll = 0.174 + (0.01 * count), pitch = 0.174 * count, yaw = 0.174 * count;
        roll_speed = 0.08 + (0.01 * count);
        pitch_speed = 0.08 + (0.01 * count);
        yaw_speed = 0.08 + (0.01 * count);
        mavlink_msg_attitude_pack_chan(_vehicleSystemId, _vehicleComponentId, _mavlinkChannel, &msg,
                                       get_boot_time(), roll, pitch, yaw, roll_speed, pitch_speed, yaw_speed);
        respondWithMavlinkMessage(c, msg);
        local_ned_x = 0.0 + (0.01 * count), local_ned_y = 0.0 * count, local_ned_z = -0.0 * count;
        ned_vx = 0.0 + (0.01 * count);
        ned_vy = 0.0 + (0.01 * count);
        ned_vz = 0.0 + (-0.01 * count);
        mavlink_msg_local_position_ned_pack_chan(_vehicleSystemId, _vehicleComponentId, _mavlinkChannel,
                                                 &msg, get_boot_time(), local_ned_x, local_ned_y,local_ned_z,
                                                 ned_vx, ned_vy, ned_vz);
        sleep(1);
        count ++ ;
        if(count > 100){ count = 0;}
    }
 }
int main(int argc, char* argv[]) {
    // Create a client endpoint
    client c;

    std::string uri = "ws://192.168.51.135:9002";

    if (argc == 2) {
        uri = argv[1];
    }
    start = std::chrono::system_clock::now();

    try {
        // Set logging to be pretty verbose (everything except message payloads)
        c.set_access_channels(websocketpp::log::alevel::all);
        c.clear_access_channels(websocketpp::log::alevel::frame_payload);
        c.clear_access_channels(websocketpp::log::alevel::frame_header);
        c.clear_error_channels(websocketpp::log::elevel::all);
        // Initialize ASIO
        c.init_asio();

        // Register our message handler
        c.set_message_handler(bind(&on_message, &c, ::_1, ::_2));

//        c.set_message_handler(bind(&heart_messages, &c, ::_1,::_2));
        websocketpp::lib::error_code ec;
        client::connection_ptr con = c.get_connection(uri, ec);
        if (ec) {
            std::cout << "could not create connection because: " << ec.message() << std::endl;
            return 0;
        }
        // Note that connect here only requests a connection. No network messages are
        // exchanged until the event loop starts running in the next line.
        c.connect(con);

        // Start the ASIO io_service run loop
        // this will cause a single connection to be made to the server. c.run()
        // will exit when this connection is closed.
        c.run();
        thread mav_msg_send(bind(heart_messages,&c));
        mav_msg_send.join();
    } catch (websocketpp::exception const & e) {
        std::cout << e.what() << std::endl;
    }
}
