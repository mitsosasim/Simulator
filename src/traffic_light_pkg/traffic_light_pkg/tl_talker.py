#!/usr/bin/env python3

#
#  *****************************************************************************************
#  * @file     tl_talker.py
#  * @author   RBRO/PJ-IU
#  * @version  V1.0
#  * @date     08-12-2020 drago
#  * @brief    This file contains the traffic light operation for the start, tl1,tl2 and tl3
#  *****************************************************************************************
#

import rclpy
import rclpy.time

from std_msgs.msg import Byte
from itertools import cycle
from enum import IntEnum

class trafficlight():
    # TrafficLightColor
    class Color(IntEnum):
        RED     = 0
        YELLOW  = 1
        GREEN   = 2

    def mirrorLight(self, number):
        if number == 0:
            return 2
        if number == 2:
            return 0
        return number

    
    def __init__(self):
        #Constants
        #Time for traffic light to change colors in seconds
        self.TL_interval = rclpy.time.Duration(seconds=1)

        #Initialize the node
        rclpy.init()
        node = rclpy.create_node('traffic_light_publisher_node')

        self.trafficlights = []
        #Create a new publisher, specify the topic name, type of message and queue size
        tlma = node.create_publisher(Byte, '/automobile/trafficlight/master', 1)
        tlsl = node.create_publisher(Byte, '/automobile/trafficlight/slave', 1)
        tlam = node.create_publisher(Byte, '/automobile/trafficlight/antimaster', 1)
        tlst = node.create_publisher(Byte, '/automobile/trafficlight/start', 1)
        
        self.trafficlights.append(tlma)
        self.trafficlights.append(tlsl)
        self.trafficlights.append(tlam)
        self.trafficlights.append(tlst)

        self.node = node
        self.rate = node.create_rate(10)

    #Function that publishes into the TL Topic the TL message (id and state)
    def sendState(self, id, state):
        self.trafficlights[id].publish(Byte(data=state.to_bytes(1)))

    def run(self):
        # The middle intersection 
        self.pattern = [
            self.Color.RED,
            self.Color.RED,
            self.Color.RED,
            self.Color.RED,
            self.Color.RED,

            self.Color.YELLOW,
            self.Color.YELLOW,
            
            self.Color.GREEN,
            self.Color.GREEN,
            self.Color.GREEN,
            self.Color.GREEN,
            self.Color.GREEN,

            self.Color.YELLOW,
            self.Color.YELLOW
        ]        
    
        # Cycles of patterns
        self.maincycle       = cycle(self.pattern)      
        self.last_time = self.node.get_clock().now() - self.TL_interval - self.TL_interval

        while rclpy.ok():
            current_time = self.node.get_clock().now()
            if current_time - self.last_time > self.TL_interval:
                self.main_state = next(self.maincycle)
                self.last_time  = current_time

            # Send State for the two opposite traffic lights
            self.sendState(0, self.main_state)
            self.sendState(1, self.main_state)

            # Send State for traffic light with the opposite state of the previous ones
            self.sendState(2, self.mirrorLight(self.main_state))

            #Send State for the start semaphore
            self.sendState(3, self.mirrorLight(self.main_state))

            self.rate.sleep() #publish at 10hz


def main():
    nod = trafficlight()
    nod.run()


if __name__ == '__main__':
    nod = trafficlight()
    nod.run()
