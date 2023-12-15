import pygame
from robomaster import robot, camera, robotic_arm
import time
import pandas as pd
import numpy as np
import csv

# Initialize RoboMaster EP
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
version_ = ep_robot.get_version()
print(f"Robot Version {version_}")

if version_ == None:
    ep_robot.close()
else:
    # Initializing chassis, camera, joystick
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_720P)
    pygame.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    all_info = np.empty((0, 6), dtype=int) # all information list
    t_v = np.array([])
    gripper_status = str()
    current_gripper_action = str()

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
    # print(f"x={t_v[0]} y={t_v[1]} z={t_v[2]} yaw={t_v[3]} pitch={t_v[4]} roll={t_v[5]}")
    all_info = np.append(all_info, [t_v], axis=0)
ep_chassis.sub_attitude(freq=5, callback=sub_attitude_info_handler)

# subscribe to gripper data (this functional!)
def gripper_data_handler(grip_stat_info):
    global gripper_status
    gripper_status= grip_stat_info
ep_gripper.sub_status(freq=5, callback=gripper_data_handler)

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION or event.type == pygame.JOYHATMOTION or event.type == pygame.JOYBUTTONDOWN:
                pygame.event.pump()
                # driving
                axis_x = joystick.get_axis(1)
                axis_y = joystick.get_axis(0)
                axis_z = joystick.get_axis(3)
                scaler = 0.2 if joystick.get_button(0) else 1
                ep_robot.chassis.drive_speed(x=-axis_x*scaler,y=axis_y*0.5*scaler,z=axis_z*100*scaler,timeout=0)

                ep_robot.play_audio(filename="bomb_countdown.wav") if joystick.get_button(5) else None
                ep_robot.play_audio(filename="mexican_hat_dance.wav") if joystick.get_button(6) else None
                # ep_robot.play_audio(filename="hola.wav") if joystick.get_button(7) else None

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

# Stop the program (Ctrl+C) and end subscriptions
except KeyboardInterrupt:
    # main_df = pd.DataFrame(all_info, columns = ['x', 'y', 'z', 'yaw', 'pitch', 'roll'])
    # main_df.to_csv('all_data_history.csv', index=False)
    # print("All Data: ", all_info)
    # print("Dataframe: ", main_df)
    ep_gripper.unsub_status()
    ep_arm.unsub_position()
    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()
    ep_camera.stop_video_stream()
    ep_robot.close()
    pygame.quit()
