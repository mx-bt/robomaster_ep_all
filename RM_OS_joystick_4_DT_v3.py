"""
Robomaster EP Core Operating System
Subject focus: ICT Systems and MBTM Project

Working Functionality:
- driving with joystick
- arm and gripper movement with joystick
- route tracking and live strean (use separate file: animated_plot.py)
- computer vision for persons and robots
- optional ap and sta connection
- some testing functionalities for precision testing
- optional fun mode

Counterintuitive key features:
- stabilizes itself when not moving
- gripper status detection (open, close, chill)
- 5 different arm positions
"""

import pygame
from robomaster import robot, camera, robotic_arm,conn
import time 
import pandas as pd
import numpy as np
import csv
from MyQR import myqr
from PIL import Image

# configuration
activate_CV = False  # choose between "cv" and "no_cv"
activate_live_tracking = True
sta_connection = False # choose between "sta" and "ap"
fun_mode = False 

ep_robot = robot.Robot()
# Initialize RoboMaster EP

if sta_connection == True:
    # make sure that the robot is connected to the same network as the computer
    ep_robot.initialize(conn_type="sta", sn="3JKCK7W0030DCD")
else:
    ep_robot.initialize(conn_type="ap")


ep_version = ep_robot.get_version()
print(f"Robot Version {ep_version}")


if ep_version == None:
    ep_robot.close()
    print("Robot not correctly connected")
else:
    # Initializing chassis, camera, joystick
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    pygame.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    all_info = np.empty((0, 6), dtype=int) # all information list
    t_v = np.array([])
    gripper_status = str()
    current_gripper_action = str()


# ==============================================================
# CV Functionality
# ==============================================================
if activate_CV == True:
    import threading
    import cv2 # OpenCV Python binary extension loader
    ep_vision = ep_robot.vision

    class PersonInfo:

        def __init__(self, x, y, w, h):
            self._x = x
            self._y = y
            self._w = w
            self._h = h

        @property
        def pt1_person(self):
            return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)

        @property
        def pt2_person(self):
            return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)

        @property
        def center_person(self):
            return int(self._x * 1280), int(self._y * 720)
    persons = []

    class RobotInfo:
        def __init__(self, x, y, w, h):
            self._x = x
            self._y = y
            self._w = w
            self._h = h

        @property
        def pt1_robot(self):
            return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)

        @property
        def pt2_robot(self):
            return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)

        @property
        def center_robot(self):
            return int(self._x * 1280), int(self._y * 720)
    robots = []

    cv_lock = threading.Lock()

    def on_detect_persons(person_info):

        global persons

        with cv_lock:
            num_persons = len(person_info)
            persons.clear()
            for i in range(0, num_persons):
                x, y, w, h = person_info[i]
                persons.append(PersonInfo(x, y, w, h))
                # print("person: x:{0}, y:{1}, w:{2}, h:{3}".format(x, y, w, h))

    def on_detect_robots(robots_info):

        global robots

        with cv_lock:
            num_robots = len(robots_info)
            robots.clear()
            for i in range(0, num_robots):
                x, y, w, h = robots_info[i]
                robots.append(RobotInfo(x, y, w, h))
                # print("robot: x:{0}, y:{1}, w:{2}, h:{3}".format(x, y, w, h))

    def process_and_display_image(ep_camera, robots): #persons,
        try: 
            while True:
                img = ep_camera.read_cv2_image(strategy="newest", timeout=5)
                with cv_lock:
                    for r in range(0, len(robots)):
                        pt1_robot = robots[r].pt1_robot
                        pt2_robot = robots[r].pt2_robot
                        cv2.rectangle(img, pt1_robot, pt2_robot, (255, 255, 255))

                        text = f"Robot {r+1}"
                        org = (pt1_robot[0], pt2_robot[1] - 10)  # Position the text slightly above the bounding box
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 0.5
                        font_thickness = 1
                        cv2.putText(img, text, org, font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)

                    # for p in range(0, len(persons)):
                    #     pt1_person = persons[p].pt1_person
                    #     pt2_person = persons[p].pt2_person
                    #     cv2.rectangle(img, pt1_person, pt2_person, (255, 255, 255))
                        
                    #     text = f"Person {p+1}"
                    #     org = (pt1_person[0], pt2_person[1] - 10)  # Position the text slightly above the bounding box
                    #     font = cv2.FONT_HERSHEY_SIMPLEX
                    #     font_scale = 0.5
                    #     font_thickness = 1
                    #     cv2.putText(img, text, org, font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)

                    cv2.imshow("computer_vision_robot", img)
                    cv2.waitKey(10)
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()

    ep_camera.start_video_stream(display=False)
    cv_result_robot = ep_vision.sub_detect_info(name="robot", callback=on_detect_robots)
    # cv_result_person = ep_vision.sub_detect_info(name="person", callback=on_detect_persons)
    # create thread
    image_thread = threading.Thread(target=process_and_display_image, args=(ep_camera, robots)) #robots,persons,
    # Start the thread
    image_thread.start()
elif activate_CV == False:
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_720P)
else:
    ep_robot.close()
    pygame.quit()
    print("activate_CV must be either True or False")

# ==============================================================
# Testing Functionalities
# ==============================================================
def testing_rectangle_1():
    distance = 1
    xy_speed = 1
    for _ in range(0, 4):
        ep_chassis.move(y=-distance,xy_speed=xy_speed).wait_for_completed()
        ep_chassis.move(x=-distance,xy_speed=xy_speed).wait_for_completed()
        ep_chassis.move(y=distance,xy_speed=xy_speed).wait_for_completed()
        ep_chassis.move(x=distance,xy_speed=xy_speed).wait_for_completed()

def testing_rectangle_2():
    xy_speed = 1
    distance = 0.5
    z_speed = 90
    for _ in range(0,4):
        for _ in range(0, 4):
            ep_chassis.move(z=-90,z_speed=z_speed).wait_for_completed()
            ep_chassis.move(x=distance,xy_speed=xy_speed).wait_for_completed()



# ==============================================================´
# Live Tracking Functionalities
# ==============================================================

if activate_live_tracking == True:
    # init the csv
    with open('xy_data.csv', 'w') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=["x_value", "y_value"])
        csv_writer.writeheader()

    # Subscribe to chassis location and attitude information
    def sub_position_handler(position_info):
        global t_v
        x_v, y_v, z_v = position_info
        x_v, y_v, z_v = round(x_v,2), round(y_v,2), round(z_v,2)
        t_v = np.array([])
        t_v = np.append(t_v,[x_v,y_v,z_v])
        # print(f"chassis position: x:{x}, y:{y}, z:{z}")

        with open('xy_data.csv', 'a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=["x_value", "y_value"])
            info = {
                "x_value": x_v,
                "y_value": y_v,
            }
            csv_writer.writerow(info)
    ep_chassis.sub_position(freq=5, callback=sub_position_handler)

    def sub_attitude_info_handler(attitude_info):
        global all_info
        global t_v
        yaw, pitch, roll = attitude_info
        yaw, pitch, roll = round(yaw,2), round(pitch,2), round(roll,2)
        t_v = np.append(t_v,[yaw, pitch, roll])
        print(f"x={t_v[0]} y={t_v[1]} z={t_v[2]} yaw={t_v[3]} pitch={t_v[4]} roll={t_v[5]}")
        all_info = np.append(all_info, [t_v], axis=0)
    ep_chassis.sub_attitude(freq=5, callback=sub_attitude_info_handler)
else:
    pass    

# subscribe to gripper data (!)
def gripper_data_handler(grip_stat_info):
    "this has functional necessity for the grippper to work"
    global gripper_status
    gripper_status= grip_stat_info
ep_gripper.sub_status(freq=5, callback=gripper_data_handler)

# ==============================================================
# Main Loop for steering and command receipt
# ==============================================================

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION or event.type == pygame.JOYHATMOTION or event.type == pygame.JOYBUTTONDOWN:
                pygame.event.pump()
                # driving
                axis_x = joystick.get_axis(1)
                axis_y = joystick.get_axis(0)
                axis_z = joystick.get_axis(3)
                axis_x, axis_y, axis_z = round(axis_x,2), round(axis_y,2), round(axis_z,2)
                scaler = 0.2 if joystick.get_button(0) else 1
                ep_robot.chassis.drive_speed(x=-axis_x*scaler,y=axis_y*0.5*scaler,z=axis_z*100*scaler,timeout=0)


                if fun_mode == True:
                    ep_robot.play_audio(filename="countdown.wav") if joystick.get_button(5) else None
                    ep_robot.play_audio(filename="mexican_hat_dance.wav") if joystick.get_button(6) else None
                else:
                    testing_rectangle_1() if joystick.get_button(5) else None
                    testing_rectangle_2() if joystick.get_button(6) else None

                # arm movement
                j_hat = round(joystick.get_hat(0)[1])
                ep_arm.move(x=j_hat*180, y=(-1)*j_hat*110).wait_for_completed() if abs(j_hat) == 1 else None
                ep_arm.recenter().wait_for_completed() if joystick.get_button(1) == 1 else None

                if joystick.get_button(3) or joystick.get_button(4):
                    if joystick.get_button(3):
                        current_gripper_action = "closing..."
                        ep_gripper.close(power=100)
                    elif joystick.get_button(4):
                        current_gripper_action = "opening..."
                        ep_gripper.open(power=100)
                    else:
                        pass
                else:
                    current_gripper_action = "chilling..."
                    pass

# ==============================================================
# Exit Program and unsubscribe all subscriptions
# ==============================================================

except KeyboardInterrupt:

    if activate_CV == True:
        result = ep_vision.unsub_detect_info(name="robot") # CV
        cv2.destroyAllWindows() # CV 2
    if activate_live_tracking == True:
        # print("All Data: ", all_info)
        # main_df = pd.DataFrame(all_info, columns = ['x', 'y', 'z', 'yaw', 'pitch', 'roll'])
        # main_df.to_csv('all_data_history.csv', index=False)
        # print("Dataframe: ", main_df)
        ep_chassis.unsub_attitude()
        ep_chassis.unsub_position()
    
    ep_gripper.unsub_status()
    ep_camera.stop_video_stream()

    ep_robot.close()
    pygame.quit()
