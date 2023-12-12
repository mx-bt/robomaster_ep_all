from robomaster import robot
import pygame


ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
print(f"Robot Version {ep_robot.get_version()}")
ep_arm = ep_robot.robotic_arm
ep_arm.recenter().wait_for_completed()


pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

def arm_move(goal=None, amountv=10, amounth=10):
    if goal == -1:
        for _ in range(0,1):
            ep_arm.move(x=-amounth, y=amountv).wait_for_completed()
    elif goal == 1:
        for _ in range(0,1):
            ep_arm.move(x=amounth, y=-amountv).wait_for_completed()
    else:
        pass


try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYHATMOTION:
                hat = joystick.get_hat(0)
                arm_move(goal=hat[1], micromove=hat[0] amounth=180, amountv=110)
except KeyboardInterrupt:
    ep_robot.close()