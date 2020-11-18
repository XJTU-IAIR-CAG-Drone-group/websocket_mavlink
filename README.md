# websocket_mavlink
websocket_mavlink client and server

## user guide   
## dependencies    
ubuntu16.04, boost 1.58.0, cmake 3.5.1    
### 1. compile code with cmake
```   
mkdir build
cd build
cmake ..
make
```   
### 2. launch server on localhost:9002
```
./websocket_server
```
### 3. launch client    
**NOTE ！** *change ip to your own machine ip(running websocket_server in step 2)*
```
./websocket_client ws://ip:9002
```
