#!/usr/bin/env python3
import rclpy
import random
from functools import partial
##TODO: include the library for a msg typed VectorStamped



##TODO: complete your callback function.
def callback(msg, offsets):
    ##TODO: Add an offset to the incoming x and z coordinates.
    

    ##TODO: Print the updated coordinates.
    


def listener():
    ##TODO: Initialize the subscriber node.
    

    ##TODO: Create two random numbers between 0 and 1.
    

    ##TODO: Define the subscriber with the offset arguments
    


def main():
    try:
        listener()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
