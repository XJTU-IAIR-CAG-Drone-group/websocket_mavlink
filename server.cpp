// The ASIO_STANDALONE define is necessary to use the standalone version of Asio.
// Remove if you are using Boost Asio.
#define ASIO_STANDALONE
 
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
 
#include <functional>
 
typedef websocketpp::server<websocketpp::config::asio> server;
 
class utility_server {
public:
    utility_server() {
         // Set logging settings
        m_endpoint.set_error_channels(websocketpp::log::elevel::all);
        m_endpoint.set_access_channels(websocketpp::log::alevel::all ^ websocketpp::log::alevel::frame_payload);
 
        // Initialize Asio
        m_endpoint.init_asio();
    }
 
    void run() {
        std::cout << "Listen on port 9002" <<std::endl;
        // Listen on port 9002
        m_endpoint.listen(9002);
 
        // Queues a connection accept operation
        m_endpoint.start_accept();
 
        // Start the Asio io_service run loop
        m_endpoint.run();
    }
private:
    server m_endpoint;
};
 
int main() {
    utility_server s;
    s.run();
    return 0;
}
