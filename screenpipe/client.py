''' client.py

This file is used to: 
    1. receive image data from torcs
    2. store data in hdf5 format

Last modified: 2016-09-27: 20:31
'''
import zmq
import torcs_data_pb2
import sys
import matplotlib.pyplot as plt
import numpy as np
import cv2
import h5py
import ipdb

context = zmq.Context()
socket = context.socket(zmq.REQ)
port = "5555"
socket.connect("tcp://localhost:%s" %port)
print "Connecting to server..."

# Set up protobuf class
serialized_data = torcs_data_pb2.TorcsData()

image_dataset = []
# h5_dataset_name = "/home/torcs_dataset/dataset.hdf5"
# hf = h5py.File(h5_dataset_name, "wb")

while True:
    # Receive data and parse it
    socket.send("LD")
    # ipdb.set_trace()
    message = socket.recv()
    serialized_data.ParseFromString(message)

    
    width = list(serialized_data.width)[0]
    height = list(serialized_data.height)[0]
    save_falg = list(serialized_data.save_flag)[0]
    # steer_angle = list(serialized_data.steer_angle)[0]
    red_array = list(serialized_data.red)[0]
    green_array = list(serialized_data.green)[0]
    blue_array = list(serialized_data.blue)[0]
    red = np.empty(len(red_array), dtype = np.uint8)
    green = np.empty(len(green_array), dtype = np.uint8)
    blue = np.empty(len(blue_array), dtype = np.uint8)

    # image_array = list(serialized_data.image)[0]
    # image = np.empty(len(image_array), dtype = np.uint8)


    # Convert uchar to uint8
    for j in range(width * height):
        red[j] = np.uint8(ord(red_array[j]))
        green[j] = np.uint8(ord(green_array[j]))
        blue[j] = np.uint8(ord(blue_array[j]))

    red = red.reshape(height, width)
    green = green.reshape(height, width)
    blue = blue.reshape(height, width)
    image = cv2.merge([red, green, blue])

    # image = image.reshape(width, height, 3)
    plt.imshow(image)
    plt.show()
    print "[width, height] = [{}, {}]".format(width, height)
    # print "---steer_angle = {}".format(steer_angle)

    # image_dataset.append(image)
    # steer_angle_dataset.append(steer_angle)
    # if save_falg == 1:
        # image_np_dataset = np.array(image_dataset)
        # steer_angle_np_dataset = np.array(steer_angle_dataset)
        # print "****image_np_dataset.shape =", image_np_dataset.shape
        # print "****steer_angle_np_dataset =", steer_angle_np_dataset.shape
        # hf.create_dataset('h5_image', data = image_np_dataset)
        # hf.create_dataset('h5_steer_angle_dataset', data = steer_angle_np_dataset)
        # print "dataset saved at: " + h5_dataset_name
        # break
        


    