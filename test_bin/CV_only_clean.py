import pygame
from robomaster import robot, camera, robotic_arm
import time
import pandas as pd
import numpy as np
import csv
import cv2

# Initialize RoboMaster EP
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
version_ = ep_robot.get_version()
print(f"Robot Version {version_}")

class PersonInfo:
    def __init__(self, x, y, w, h):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
    @property
    def pt1(self):
        return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)
    @property
    def pt2(self):
        return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)
    @property
    def center(self):
        return int(self._x * 1280), int(self._y * 720)
persons = []
def on_detect_person(person_info):
    number = len(person_info)
    persons.clear()
    for i in range(0, number):
        x, y, w, h = person_info[i]
        persons.append(PersonInfo(x, y, w, h))
        print("person: x:{0}, y:{1}, w:{2}, h:{3}".format(x, y, w, h))

class RobotInfo:
    def __init__(self, x, y, w, h):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
    @property
    def pt1(self):
        return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)
    @property
    def pt2(self):
        return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)
    @property
    def center(self):
        return int(self._x * 1280), int(self._y * 720)
robots = []
def on_detect_robot(robot_info):
    number = len(robot_info)
    robots.clear()
    for i in range(0, number):
        x, y, w, h = robot_info[i]
        robots.append(RobotInfo(x, y, w, h))
        print("robot: x:{0}, y:{1}, w:{2}, h:{3}".format(x, y, w, h))


ep_vision = ep_robot.vision
ep_camera = ep_robot.camera

try:
    ep_camera.start_video_stream(False)
    # result = ep_vision.sub_detect_info(name="person", callback=on_detect_person)

    # for i in range(0, 500):
    #     img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
    #     for j in range(0, len(persons)):
    #         cv2.rectangle(img, persons[j].pt1, persons[j].pt2, (255, 255, 255))
    #     cv2.imshow("Persons", img)
    #     cv2.waitKey(1)

    result2 = ep_vision.sub_detect_info(name="person", callback=on_detect_person)
    # result1 = ep_vision.sub_detect_info(name="robot", callback=on_detect_robot)

    for i in range(0, 500):
        img = ep_camera.read_cv2_image(strategy="newest", timeout=5)
        for j in range(0, len(robots)):
            cv2.rectangle(img, robots[j].pt1, robots[j].pt2, (255, 255, 255))
        for p in range(0, len(persons)):
            cv2.rectangle(img, persons[p].pt1, persons[p].pt2, (255, 255, 255))
        cv2.imshow("cv", img)
        cv2.waitKey(1)

    cv2.destroyAllWindows()

except KeyboardInterrupt:
    pass

cv2.destroyAllWindows()
result2 = ep_vision.unsub_detect_info(name="person")
result1 = ep_vision.unsub_detect_info(name="robot")
cv2.destroyAllWindows()
ep_camera.stop_video_stream()
ep_robot.close()
