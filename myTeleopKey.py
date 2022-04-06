#!/usr/bin/env python3
#Script de python del nodo tipo Teleop_key para mover turtle

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportRelative
import termios, sys, os
from numpy import pi

TERMIOS = termios

def getkey(): #Captura la letra que ingresa del teclado
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    c = str(c).replace('b', "").replace('\'', "")
    return c
    

def teleport(x, y, ang, mode): #Servicio teleport para cambio de posición (absoluto o relativo)
    if mode == "a":
        service = "/turtle1/teleport_absolute"
        serviceMode = TeleportAbsolute
    else: 
        service = "/turtle1/teleport_relative"
        serviceMode = TeleportRelative
    rospy.wait_for_service(service)
    try:
        teleportA = rospy.ServiceProxy(service, serviceMode)
        if mode == "a":  resp1 = teleportA(x, y, ang)
        else:  resp1 = teleportA(x, ang)
        print('Teleported to x: {}, y: {}, ang: {}'.format(str(x),str(y),str(ang)))
    except rospy.ServiceException as e:
        print(str(e))

def pubVel(mov, rot): #Comunicación con el tópico de velocidad. Se publica a ese tópico
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('velPub', anonymous=False)
    vel = Twist()
    rate = rospy.Rate(10) 
    vel.linear.x = mov
    vel.angular.z = rot
    rospy.loginfo(vel)
    pub.publish(vel)
    rate.sleep()

if __name__ == '__main__':
    try:
        teleport(5.5, 5.5, pi/2, "a")
        pubVel(0, 0)
        while not rospy.is_shutdown():
            entrada = getkey()
            if entrada == "w":
                print("Key: w")
                pubVel(1, 0)
            elif entrada == "s":
                print("Key: s")
                pubVel(-1, 0)
            elif entrada == "d":
                print("Key: d")
                pubVel(0, -pi/4)
            elif entrada == "a":
                print("Key: a")
                pubVel(0, pi/4)
            elif entrada == "r":
                print("Key: r")
                teleport(5.5, 5.5, pi/2, "a")
            elif entrada == " ":
                print("Key: space")
                teleport(0, 0, pi, "r")
            else:
                print("Key not recognized")
    except rospy.ROSInterruptException:
        print("Ups, error")
        pass