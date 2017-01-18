# Prerequisites

- OpenCV
```shell
sudo apt install libopencv-dev python-opencv
```

- ZMQ
```shell
deb http://httpredir.debian.org/debian/ experimental main contrib non-free
deb-src http://httpredir.debian.org/debian/ experimental main contrib non-free
sudo apt update
sudo apt install libzmq5-dev 
```

- Protobuf
```shell
sudo apt install libprotobuf-dev python-protobuf 
```

- Matplotlib
```shell
sudo apt install python-matplotlib
```

- Anaconda packages installation
```
conda install -c conda-forge ipdb=0.10.1
conda install -c menpo opencv3=3.1.0
conda install -c anaconda protobuf=3.0.0
```

# Files changes compared to original (patched) torcs-1.3.7
- torcs-1.3.7/src/libs/raceengineclient/raceengine.cpp
- torcs-1.3.7/src/linux/main.cpp

# How to build
Rebuild TORCS
```shell
cd torcs-1.3.7
./configure
make
sudo make install
sudo make datainstall
```
Compile memory sharing source
```shell
cd torcs-1.3.7/screenpipe
g++ IPC_command.cpp torcs_data.pb.cc -o IPC_command `pkg-config --cflags --libs opencv protobuf libzmq`
```

# How to execute
Run torcs
```shell
torcs
```
Start memory sharing (initially paused)
```shell
cd torcs-1.3.7/screenpipe
./IPC_command # window "Image from TORCS" appears
```
Run Python client that receives screen images
```shell
python screenpipe_client.py
```
Press `P` (unpause) to start sharing images in the window named "Image from TORCS"

# Troubleshooting
In case protobuf requires to recompile `torcs_data_pb2.py` and `torcs_data.pb.cc`:
```shell
cd screenpipe
protoc --python-out=$PWD torcs_data.proto
protoc --cpp_out=$PWD torcs_data.proto 
```
