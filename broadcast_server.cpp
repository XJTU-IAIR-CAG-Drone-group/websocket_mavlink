#include <websocketpp/config/asio_no_tls.hpp>

#include <websocketpp/server.hpp>
#include <websocketpp/frame.hpp>
#include <iostream>
#include <set>

/*#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>*/
#include <websocketpp/common/thread.hpp>

#include <common/mavlink.h>
#include <chrono>
typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::connection_hdl;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

using websocketpp::lib::thread;
using websocketpp::lib::mutex;
using websocketpp::lib::lock_guard;
using websocketpp::lib::unique_lock;
using websocketpp::lib::condition_variable;
auto start = std::chrono::system_clock::now();
//auto end = std::chrono::system_clock::now();
/* on_open insert connection_hdl into channel
 * on_close remove connection_hdl from channel
 * on_message queue send to all channels
 */

enum action_type {
    SUBSCRIBE,
    UNSUBSCRIBE,
    MESSAGE
};

struct action {
    action(action_type t, connection_hdl h) : type(t), hdl(h) {}
    action(action_type t, connection_hdl h, server::message_ptr m)
      : type(t), hdl(h), msg(m) {}

    action_type type;
    websocketpp::connection_hdl hdl;
    server::message_ptr msg;
};

class broadcast_server {
public:
    broadcast_server() {
        start = std::chrono::system_clock::now();
        // Initialize Asio Transport
        m_server.clear_access_channels(websocketpp::log::alevel::all);
        m_server.clear_error_channels(websocketpp::log::elevel::all);
        m_server.set_access_channels(websocketpp::log::alevel::connect);
        m_server.set_access_channels(websocketpp::log::alevel::disconnect);
        m_server.init_asio();

        // Register handler callbacks
        m_server.set_open_handler(bind(&broadcast_server::on_open,this,::_1));
        m_server.set_close_handler(bind(&broadcast_server::on_close,this,::_1));
        m_server.set_message_handler(bind(&broadcast_server::on_message,this,::_1,::_2));

    }

    void run(uint16_t port) {
        // listen on specified port
        m_server.listen(port);

        // Start the server accept loop
        m_server.start_accept();

        // Start the ASIO io_service run loop
        try {
            m_server.run();
        } catch (const std::exception & e) {
            std::cout << e.what() << std::endl;
        }
    }

    void on_open(connection_hdl hdl) {
        {
            lock_guard<mutex> guard(m_action_lock);
            //std::cout << "on_open" << std::endl;
            m_actions.push(action(SUBSCRIBE,hdl));
        }
        m_action_cond.notify_one();
    }

    void on_close(connection_hdl hdl) {
        {
            lock_guard<mutex> guard(m_action_lock);
            //std::cout << "on_close" << std::endl;
            m_actions.push(action(UNSUBSCRIBE,hdl));
        }
        m_action_cond.notify_one();
    }

    void on_message(connection_hdl hdl, server::message_ptr msg) {
        // queue message up for sending by processing thread
        {
            lock_guard<mutex> guard(m_action_lock);
            //std::cout << "on_message" << std::endl;
            m_actions.push(action(MESSAGE,hdl,msg));
        }
        m_action_cond.notify_one();
    }
    void respondWithMavlinkMessage(connection_hdl hdl, const mavlink_message_t& msg)
    {
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    
        int len = mavlink_msg_to_send_buffer(buffer, &msg);
        websocketpp::lib::error_code ec;
        m_server.send(hdl, buffer, len, websocketpp::frame::opcode::binary, ec);
        if (ec) {
            std::cout << "Echo failed because: " << ec.message() << std::endl;
        }
    }
    
    int get_boot_time(){
        auto end = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        return duration.count();
    }
    void get_mavlink_home_position(mavlink_message_t &msg){
        float bogus[4];
        bogus[0] = 0.0f;
        bogus[1] = 0.0f;
        bogus[2] = 0.0f;
        bogus[3] = 0.0f;
        double _vehicleLatitude = 107.40;
        double _vehicleLongitude = 33.42;
        double _vehicleAltitude = 1.5;
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
    }
    void get_mavlink_sys_status(mavlink_message_t &msg){
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
    }
    void get_mavlink_attitude(mavlink_message_t &msg){
        roll = 0.174 + (0.01 * count), pitch = 0.174 * count, yaw = 0.174 * count;
        roll_speed = 0.08 + (0.01 * count);
        pitch_speed = 0.08 + (0.01 * count);
        yaw_speed = 0.08 + (0.01 * count);
        mavlink_msg_attitude_pack_chan(_vehicleSystemId, _vehicleComponentId, _mavlinkChannel, &msg,
                                       get_boot_time(), roll, pitch, yaw, roll_speed, pitch_speed, yaw_speed);
    }
    void get_mavlink_local_position_ned(mavlink_message_t &msg){
        local_ned_x = 0.0 + (0.01 * count), local_ned_y = 0.0 * count, local_ned_z = -0.0 * count;
        ned_vx = 0.0 + (0.01 * count);
        ned_vy = 0.0 + (0.01 * count);
        ned_vz = 0.0 + (-0.01 * count);
        
        mavlink_msg_local_position_ned_pack_chan(_vehicleSystemId, _vehicleComponentId, _mavlinkChannel,
                                                 &msg, get_boot_time(), local_ned_x, local_ned_y,local_ned_z,
                                                 ned_vx, ned_vy, ned_vz);
    }
    void get_mavlink_heartbeat(mavlink_message_t &msg){
        mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_STANDBY);
        uint8_t buf[512];
        uint16_t len;
        len = mavlink_msg_to_send_buffer(buf, &msg);
    }
    void heart_messages(){
        mavlink_message_t msg;
        
        while(1){
            if(m_connections.empty()){
                continue;
            } else {
//                lock_guard<mutex> guard(m_connection_lock);
                unique_lock<mutex> lock(m_connection_lock);
                std::cout << "[--------------connection size:"<< m_connections.size() 
                          << "--------------]:"<< std::endl;              
                con_list::iterator it;
                for (it = m_connections.begin(); it != m_connections.end(); ++it) {
                    //mavlink heartbeat
                    get_mavlink_heartbeat(msg);
                    respondWithMavlinkMessage(*it, msg);
    #ifdef dronestate_sim_send        
                    //-------------home position---------------//
                    get_mavlink_home_position(msg);
                    respondWithMavlinkMessage(*it, msg);
                    //-------------SysStatus---------------//
                    get_mavlink_sys_status(msg);
                    respondWithMavlinkMessage(*it, msg);
                    //-------------attitude---------------//
                    get_mavlink_attitude(msg);
                    respondWithMavlinkMessage(*it, msg);
                    //-------------local_position_ned---------------//
                    get_mavlink_local_position_ned(msg);
                    respondWithMavlinkMessage(*it, msg);
                    sleep(1);
                    count ++ ;
                    if(count > 100){ count = 0;}
     #endif       
                }
                lock.unlock();
//                sleep(1);
            }
        }
    }

    void process_messages() {
        while(1) {
            unique_lock<mutex> lock(m_action_lock);

            while(m_actions.empty()) {
                m_action_cond.wait(lock);
            }

            action a = m_actions.front();
            m_actions.pop();

            lock.unlock();

            if (a.type == SUBSCRIBE) {
//                lock_guard<mutex> guard(m_connection_lock);
                unique_lock<mutex> connection_lock(m_connection_lock);
                m_connections.insert(a.hdl);
                connection_lock.unlock();
            } else if (a.type == UNSUBSCRIBE) {
//                lock_guard<mutex> guard(m_connection_lock);
                unique_lock<mutex> connection_lock(m_connection_lock);
                m_connections.erase(a.hdl);
                connection_lock.unlock();
            } else if (a.type == MESSAGE) {
//                lock_guard<mutex> guard(m_connection_lock);
                unique_lock<mutex> connection_lock(m_connection_lock);
//                    m_server.send(*it,a.msg);
//                m_server.send(a.hdl, buf, len, websocketpp::frame::opcode::binary);
                if(a.msg->get_opcode() == websocketpp::frame::opcode::text){
                    std::cout << "on_message called with hdl: " << a.hdl.lock().get()
                              << " and message: " << a.msg->get_payload()
                              << std::endl; 
                } else {
                    std::cout << "on_message called with hdl: " << a.hdl.lock().get()
                              << " and message: " << websocketpp::utility::to_hex(a.msg->get_payload()) 
                              << std::endl;
                }
                connection_lock.unlock();
            } else {
                //undefined
                
            }
            
        }
    }
private:
    typedef std::set<connection_hdl,std::owner_less<connection_hdl> > con_list;

    server m_server;
    con_list m_connections;
    std::queue<action> m_actions;

    mutex m_action_lock;
    mutex m_connection_lock;
    condition_variable m_action_cond;
    uint8_t _vehicleSystemId = '1' - '0';
    uint8_t _vehicleComponentId = '1' - '0';
    uint8_t _mavlinkChannel = '0' - '0';
public:
    int count = 0;
    float roll = 0.0, pitch = 0.0, yaw = 0.0;
    float roll_speed = 0.08, pitch_speed = 0.08, yaw_speed = 0.08;
    float local_ned_x = 0, local_ned_y = 0, local_ned_z = 0;
    float ned_vx = 0, ned_vy = 0, ned_vz = 0;
};

int main() {
    try {
    broadcast_server server_instance;

    // Start a thread to run the processing loop
    thread t(bind(&broadcast_server::process_messages,&server_instance));
    thread heartbeart(bind(&broadcast_server::heart_messages,&server_instance));
    // Run the asio loop with the main thread
    server_instance.run(9002);

    t.join();
    heartbeart.join();
    } catch (websocketpp::exception const & e) {
        std::cout << e.what() << std::endl;
    }
}
