#!/usr/bin/env python
'''
This is a game for testing the VR control system for the Franka Emika panda dual-arm system.
The game will set a random pose target in the span where the arm can reach. 
Then if both arms are in the respective circle,
the target collected and two new targets will be spawned (and so on).
'''

import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64, Float32, Float32MultiArray
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
import numpy as np
from math import pi, sin, sqrt
import csv
import random
import os
import sys
random.seed(7)

LEFT_ARM = 0
RIGHT_ARM = 1

class RobotData:
    def __init__(self):
        self.arms = [ArmData("left"), ArmData("right")]
        self.sub = [rospy.Subscriber("/robot/left/currentPose", PoseStamped, self.leftCurrentPassCallback, queue_size=1),
                    rospy.Subscriber("/robot/right/currentPose", PoseStamped, self.rightCurrentPoseCallback, queue_size=1)]

    def leftCurrentPassCallback(self, data):
        self.arms[LEFT_ARM].currentPose = data

    def rightCurrentPoseCallback(self, data):
        self.arms[RIGHT_ARM].currentPose = data

class ArmData:
    def __init__(self, name):
        self.currentPose = PoseStamped() 
        self.name = name

def EuclideanDistance(pose1, pose2):
    return sqrt(pow(pose1.position.x - pose2.position.x,2) +
                pow(pose1.position.y - pose2.position.y,2) + 
                pow(pose1.position.z - pose2.position.z,2))


class GameArea:
    def __init__(self, x_span, y_span ,z_span):
        self.x = x_span # Two dimensional data (from x_span = [x_0 -> x_1])
        self.y = y_span 
        self.z = z_span

class GameDataArm:
    def __init__(self, gameArea, targetPose, name, allowed_slap = 0.2):
        # Initialization of data
        self.targetPose = targetPose
        self.gameArea = gameArea
        self.name = name
        self.allowed_slap = allowed_slap
        self.score = 0

        # Publishers
        self.pub          = rospy.Publisher("/game/" + name + "/targetPose", PoseStamped, queue_size=1)
        self.pub_marker   = rospy.Publisher("/game/" + name + "/marker",     Marker,      queue_size=1)
        self.pub_gameArea = rospy.Publisher("/game/" + name + "/gameArea",   Marker,      queue_size=1)

    def setRandomPose(self):
        self.targetPose.position.x = random.uniform(self.gameArea.x[0], self.gameArea.x[1])
        self.targetPose.position.y = random.uniform(self.gameArea.y[0], self.gameArea.y[1])
        self.targetPose.position.z = random.uniform(self.gameArea.z[0], self.gameArea.z[1])

    def poseInSpan(self, pose):
        self.score = int(EuclideanDistance(pose, self.targetPose) < self.allowed_slap)
        return self.score
    
    def publishGameArea(self):
        gameAreaMarker = Marker()
        gameAreaMarker.header.frame_id = "/world"
        gameAreaMarker.header.stamp = rospy.Time.now()
        gameAreaMarker.id = 0
        gameAreaMarker.type = 1 # sphere
        gameAreaMarker.action = 0
        originPose = Pose()
        originPose.orientation.x = 0
        originPose.orientation.y = 0
        originPose.orientation.z = 0
        originPose.orientation.w = 1
        originPose.position.x = (self.gameArea.x[0] + self.gameArea.x[1])/2
        originPose.position.y = (self.gameArea.y[0] + self.gameArea.y[1])/2
        originPose.position.z = (self.gameArea.z[0] + self.gameArea.z[1])/2
        gameAreaMarker.pose = originPose
        gameAreaMarker.scale.x = self.gameArea.x[0] - self.gameArea.x[1]
        gameAreaMarker.scale.y = self.gameArea.y[0] - self.gameArea.y[1]
        gameAreaMarker.scale.z = self.gameArea.z[0] - self.gameArea.z[1]

        gameAreaMarker.color.r = 0.5
        gameAreaMarker.color.g = 0.5
        gameAreaMarker.color.b = 0.5
        gameAreaMarker.color.a = 0.5
        self.pub_gameArea.publish(gameAreaMarker)

    def publishPoseSpheres(self):
        msg = PoseStamped()
        msg.header.frame_id = "/world"
        msg.header.stamp = rospy.Time.now()
        msg.pose = self.targetPose
        self.pub.publish(msg)

        robotMarker = Marker()
        robotMarker.header.frame_id = "/world"
        robotMarker.header.stamp = rospy.Time.now()
        robotMarker.id = 0
        robotMarker.type = 2 # sphere
        robotMarker.action = 0
        robotMarker.pose = self.targetPose
        robotMarker.scale.x = self.allowed_slap
        robotMarker.scale.y = self.allowed_slap
        robotMarker.scale.z = self.allowed_slap
        robotMarker.pose.orientation.w = 1

        robotMarker.color.r = int(self.name == "left")
        robotMarker.color.g = self.score
        robotMarker.color.b = int(self.name == "right")
        robotMarker.color.a = 0.5

        self.pub_marker.publish(robotMarker)

    def publishGraphics(self):
        self.publishPoseSpheres()
        self.publishGameArea()

    def publishWinning(self):
        msg = PoseStamped()
        msg.header.frame_id = "/world"
        msg.header.stamp = rospy.Time.now()
        msg.pose = self.targetPose
        self.pub.publish(msg)

        robotMarker = Marker()
        robotMarker.header.frame_id = "/world"
        robotMarker.header.stamp = rospy.Time.now()
        robotMarker.id = 0
        robotMarker.type = 2 # sphere
        robotMarker.action = 0
        robotMarker.pose = self.targetPose
        robotMarker.scale.x = self.allowed_slap
        robotMarker.scale.y = self.allowed_slap
        robotMarker.scale.z = self.allowed_slap

        robotMarker.color.r = 0
        robotMarker.color.g = 1
        robotMarker.color.b = 0
        robotMarker.color.a = 1

        self.pub_marker.publish(robotMarker)


class GameHandler:
    def __init__(self, leftGameArea, rightGameArea, freq):
        self.arms = [GameDataArm(leftGameArea, Pose(), "left"),
                     GameDataArm(rightGameArea, Pose(), "right")]

        self.score = 0
        self.time = 0
        self.freq = freq
        self.game_data = []
        self.robotData = RobotData()
        self.updateRandomPose()

    def updateRandomPose(self):
        for arm in self.arms:
            arm.setRandomPose()

    def checkWinningCondition(self, poses):
        score = []
        for i, arm in enumerate(self.arms):
            score.append(arm.poseInSpan(poses[i]))
        return score
    
    def loop(self):
        self.time += 1.0/freq
        for arm in self.arms:
            arm.publishGraphics()
        poses = [self.robotData.arms[LEFT_ARM].currentPose.pose,
                 self.robotData.arms[RIGHT_ARM].currentPose.pose]
        score = self.checkWinningCondition(poses)
        if score[0] and score[1]:
            self.score += 1
        self.game_data.append([
            self.time,
            self.score,
            poses[LEFT_ARM].position.x, poses[LEFT_ARM].position.y, poses[LEFT_ARM].position.z,
            self.arms[LEFT_ARM].targetPose.position.x, self.arms[LEFT_ARM].targetPose.position.y, self.arms[LEFT_ARM].targetPose.position.z,
            poses[RIGHT_ARM].position.x, poses[RIGHT_ARM].position.y, poses[RIGHT_ARM].position.z,
            self.arms[RIGHT_ARM].targetPose.position.x, self.arms[RIGHT_ARM].targetPose.position.y, self.arms[RIGHT_ARM].targetPose.position.z,
        ])
        if score[0] and score[1]:
            self.updateRandomPose()
        if self.score >= 5:
            self.arms[0].publishWinning()
            self.arms[1].publishWinning()
            save_matrix = np.asarray(self.game_data)
            cwd = os.getcwd()
            print("Saved game data in matrix at: "+ cwd + "/game_data.csv")
            print("The structure of the matrix is time, score, left position, left target position, right position, right target position")
            np.savetxt(cwd + "/game_data.csv", save_matrix, delimiter=",")
            print("--------------------\n----- You won! -----\n--------------------")
            print("--------------------\n-Elapsed Time: " + str(self.time) + "-\n--------------------")
            # input("Press any key to exit...")
            exit()


if __name__=="__main__":
    if(len(sys.argv) > 1):
        random.seed(int(sys.argv[1]))
    rospy.init_node("pose_game")

    leftGameArea = GameArea([0.1,0.5],[0.1,1.0],[0.3,0.8])
    rightGameArea = GameArea([0.1,0.5],[-1.0,-0.1],[0.3,0.8])
    freq = 10.0
    gameHandler = GameHandler(leftGameArea, rightGameArea, freq)

    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        gameHandler.loop()
        rate.sleep()



