// Used to create shared memory for data and command sharing.
//
// Required files:
//      1. IPC_command.cpp
//      2. torcs_data.pb.h 
//      3. torcs_data.pb.cc
//
// $ g++ IPC_command.cpp torcs_data.pb.cc -o IPC_command`pkg-config --cflags --libs opencv protobuf libzmq`
// 

#include <iostream>
#include <unistd.h>
#include <sys/shm.h>
#include <stdlib.h>  
#include <stdio.h>
#include <opencv2/opencv.hpp>
// #include "cv.h"  
#include "opencv2/highgui/highgui.hpp"
#include <zmq.hpp> 
#include "torcs_data.pb.h"

using namespace std;
using namespace cv;

#define image_width 640
#define image_height 480
#define resize_width 640
#define resize_height 480

struct shared_use_st  
{  
    int written;
    unsigned char data[image_width*image_height*3];
    int pause;
    int zmq_flag; 
    int save_flag; 
};

int main(int argc, char const *argv[])
{
    void *shm = NULL;
    struct shared_use_st *shared;
    int shmid;
    
    shmid = shmget((key_t)1234, sizeof(struct shared_use_st), 0666|IPC_CREAT);
    if(shmid == -1)  
    {  
        fprintf(stderr, "shmget failed\n");  
        exit(EXIT_FAILURE);  
    }
    shm = shmat(shmid, 0, 0);  
    if(shm == (void*)-1)  
    {  
        fprintf(stderr, "shmat failed\n");  
        exit(EXIT_FAILURE);  
    }  
    printf("\n********** Memory sharing started, attached at blablabal %X **********\n", shm);

    shared = (struct shared_use_st*)shm; 
    shared->written = 0;
    shared->pause = 0;
    shared->zmq_flag = 0;  
    shared->save_flag = 0;


    // Setup zmq
    static zmq::context_t context(1);
    static zmq::socket_t socket(context, ZMQ_PAIR);
    printf("binding to socket\n");
    socket.bind("tcp://*:5555");
    printf("done\n");
    TorcsData torcs_data;
    unsigned char image[resize_width*resize_height * 3];

    // Setup opencv
    IplImage* screenRGB=cvCreateImage(cvSize(image_width,image_height),IPL_DEPTH_8U,3);
    IplImage* resizeRGB=cvCreateImage(cvSize(resize_width,resize_height),IPL_DEPTH_8U,3);
    cvNamedWindow("Image from TORCS",1);
    int key;
    IplImage* out_red = cvCreateImage(cvSize(resize_width,resize_height), IPL_DEPTH_8U, 1);
    IplImage* out_green = cvCreateImage(cvSize(resize_width,resize_height), IPL_DEPTH_8U, 1);
    IplImage* out_blue = cvCreateImage(cvSize(resize_width,resize_height), IPL_DEPTH_8U, 1);

    while (true) {    

        if (shared->written == 1) {

            for (int h = 0; h < image_height; h++) {
                for (int w = 0; w < image_width; w++) {
                   screenRGB->imageData[(h*image_width+w)*3+2]=shared->data[((image_height-h-1)*image_width+w)*3+0];
                   screenRGB->imageData[(h*image_width+w)*3+1]=shared->data[((image_height-h-1)*image_width+w)*3+1];
                   screenRGB->imageData[(h*image_width+w)*3+0]=shared->data[((image_height-h-1)*image_width+w)*3+2];
                }
            }
            //printf("---screenRGB read complete.\n");
            // Resize image and send it as protobuf message
            cvResize(screenRGB, resizeRGB);
            cvSplit(resizeRGB, out_blue, out_green, out_red, NULL);

            // printf("---image read complete.\n");
            torcs_data.clear_width();
            torcs_data.clear_height();
            torcs_data.clear_red();
            torcs_data.clear_green();
            torcs_data.clear_blue();
            torcs_data.clear_save_flag();
            torcs_data.add_red((const void*)out_red->imageData, (size_t) resize_width * resize_height);
            torcs_data.add_green((const void*)out_green->imageData, (size_t) resize_width * resize_height);
            torcs_data.add_blue((const void*)out_blue->imageData, (size_t) resize_width * resize_height);
            torcs_data.add_width(resize_width);
            torcs_data.add_height(resize_height);
            torcs_data.add_save_flag(shared->save_flag);
            //cout << "torcs_data shape: [" << torcs_data.width(0) << ", " << torcs_data.height(0) << "]" << endl;

            cvShowImage("Image from TORCS", resizeRGB);

            string serialized_data;
            torcs_data.SerializeToString(&serialized_data);

            /*zmq::message_t request;
            socket.recv(&request);
            std::string replyMessage = std::string(static_cast<char *>(request.data()), request.size());
            //std::cout << "Recived from client: " + replyMessage << std::endl;*/

            zmq::message_t reply(serialized_data.size());
            // std::cout << "checked OK!  2" << std::endl;
            memcpy((void*) reply.data(), serialized_data.data(), serialized_data.size());
            //std::cout << "---length of message to client: " << reply.size() << std::endl;
            socket.send(reply);

            shared->written=0;
        }

        key = cvWaitKey(100);
        if (key==1048688 || key==112){ // P
            shared->pause = 1 - shared->pause;
            printf("shared->pause = %d\n", shared->pause);
        }else if(key == 115){ // S
            shared->save_flag = 1 - shared->save_flag;
            printf("shared->save_flag = %d\n", shared->save_flag);
        }else if (key==1048603 || key==27){ // ESC
            shared->pause = 0;
            printf("ESC is pressed, exit\n");
            break;
        }


    }

    if(shmdt(shm) == -1)  
    {  
        fprintf(stderr, "shmdt failed\n");  
        exit(EXIT_FAILURE);  
    }  

    if(shmctl(shmid, IPC_RMID, 0) == -1)  
    {  
        fprintf(stderr, "shmctl(IPC_RMID) failed\n");  
        exit(EXIT_FAILURE);  
    }
    printf("\n********** Memory sharing stopped. Good Bye! **********\n");  

    return 0;
}
