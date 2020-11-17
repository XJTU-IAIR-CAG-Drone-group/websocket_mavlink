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
                    //m_server.send(*it,a.msg);
                    //mavlink heartbeat
                    mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_STANDBY);
                    uint8_t buf[512];
                    uint16_t len;
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    websocketpp::lib::error_code ec;
                    m_server.send(*it, buf, len, websocketpp::frame::opcode::binary, ec);
                    if (ec) {
                        std::cout << "Echo failed because: " << ec.message() << std::endl;
                    }
                }
                lock.unlock();
                sleep(1);
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
