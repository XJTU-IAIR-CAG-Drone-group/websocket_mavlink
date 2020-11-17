# websocket_mavlink
websocket_mavlink client and server

## user guide   
## dependencies    
ubuntu16.04, boost 1.58.0, cmake 3.5.1    
### compile code with cmake
```   
mkdir build
cd build
cmake ..
make
```   
### launch server on localhost:9002
```
./websocket_server
```
### launch client 
```
./websocket_client ws://ip:9002
```
