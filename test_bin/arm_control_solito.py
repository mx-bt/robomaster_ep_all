import pygame
from robomaster import robot
import time
from datetime import datetime

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
print(f"Robot Version {ep_robot.get_version()}")

ep_arm = ep_robot.robotic_arm
ep_gripper = ep_robot.gripper

global_arm_position = tuple()
gripper_status = str()
current_gripper_action = str()

def arm_position_handler(arm_callback):

    global global_arm_position
    # global arm_position
    pos_x, pos_y = arm_callback
    global_arm_position = tuple(pos_x, pos_y)

    # print(f"ARM position: x {pos_x} y{pos_y}")

def gripper_data_handler(grip_stat_info):

    global gripper_status

    gripper_status = grip_stat_info

    # print(f"GRIPPER status: [{gripper_status}] action: [{current_gripper_action}] ARM position: x {global_arm_position[0]} y{global_arm_position[1]}")

ep_arm.sub_position(freq=5, callback=arm_position_handler)
ep_gripper.sub_status(freq=5, callback=gripper_data_handler)

try:
    while True:
        # for event in pygame.event.get():
        #     if event.type == pygame.JOYAXISMOTION or event.type == pygame.JOYHATMOTION or event.type == pygame.JOYBUTTONDOWN:
        pygame.event.pump()  # Pump events to update joystick state

        j_hat = round(joystick.get_hat(0)[1])
        ep_arm.move(x=j_hat*180, y=(-1)*j_hat*110).wait_for_completed() if abs(j_hat) == 1 else None
        ep_arm.recenter().wait_for_completed() if joystick.get_button(1) == 1 else None

        if joystick.get_button(3) or joystick.get_button(4):
            if joystick.get_button(3):
                current_gripper_action = "closing..."
                ep_gripper.close(power=50)
            elif joystick.get_button(4):
                current_gripper_action = "opening..."
                ep_gripper.open(power=50)
            else:
                pass
        else:
            current_gripper_action = "chilling..."
            pass

        timestamp = datetime.now().strftime(r"%Y-%m-%d %H:%M:%S")
        print(f"[{timestamp}] {j_hat} gripper: {gripper_status} and {current_gripper_action}")


except KeyboardInterrupt:
    # ep_arm.unsub_position()
    ep_arm.unsub_position()
    ep_gripper.unsub_status()
    ep_robot.close()
    pygame.quit()

