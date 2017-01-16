# Prerequisites

- OpenCV
```
sudo apt install libopencv-dev python-opencv
```

- ZMQ
```
deb http://httpredir.debian.org/debian/ experimental main contrib non-free
deb-src http://httpredir.debian.org/debian/ experimental main contrib non-free
sudo apt update
sudo apt install libzmq5-dev 
```

- Protobuf
```
sudo apt install libprotobuf-dev python-protobuf 
```

- Matplotlib
```
sudo apt install python-matplotlib
```


# How to build
1. Rebuild TORCS
	```
	cd torcs-1.3.7
	./configure
	make
	sudo make install
	sudo make datainstall
	```
2. Compile memory sharing source
	```
	cd torcs-1.3.7/screenpipe
	g++ IPC_command.cpp torcs_data.pb.cc -o IPC_command `pkg-config --cflags --libs opencv protobuf libzmq`
	```

# How to execute
1. Run torcs
	```
	torcs
	```
2. Start memory sharing (initially paused)
	```
	cd torcs-1.3.7/screenpipe
	./IPC_command # window "Image from TORCS" appears
	```
3. Run Python client that receives screen images
	```
	python screenpipe_client.py```
4. Press `P` (unpause) to start sharing images in the window named "Image from TORCS"

# Troubleshooting
In case protobuf requires to recompile `torcs_data_pb2.py` and `torcs_data.pb.cc`:
```
cd screenpipe
protoc --python-out=$PWD torcs_data.proto
protoc --cpp_out=$PWD torcs_data.proto 
```
