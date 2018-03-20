import pygame
from pygame.locals import *
import time
import sys
import rospy
from geometry_msgs.msg import Twist


def main():

    # initialize pygame to get keyboard event
    pygame.init()
    window_size = Rect(0, 0, 300, 300)
    screen = pygame.display.set_mode(window_size.size)

    # initialize ros publisher
    twist_pub = rospy.Publisher('keyboard/twist', Twist, queue_size=10)
    rospy.init_node('keyboard_control')
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        # get key value and set Twist
        for event in pygame.event.get():
            if event.type == KEYDOWN:

                twist = Twist()

                # space to pause to UAV
                if event.key == pygame.K_SPACE:
                    print 'stop'

                # x,y velocity control: x axis forward, y axis right
                elif event.key == pygame.K_UP:
                    print 'forward'
                    twist.linear.x = 1.0
                elif event.key == pygame.K_DOWN:
                    print 'backward'
                    twist.linear.x = -1.0
                elif event.key == pygame.K_LEFT:
                    print 'left'
                    twist.linear.y = -1.0
                elif event.key == pygame.K_RIGHT:
                    print 'right'
                    twist.linear.y = 1.0

                # yaw and z control
                elif event.key == pygame.K_w:
                    print 'up'
                    twist.linear.z = 1.0
                elif event.key == pygame.K_s:
                    print 'down'
                    twist.linear.z = -1.0
                elif event.key == pygame.K_a:
                    print 'turn left'
                    twist.angular.z = -1.0
                elif event.key == pygame.K_d:
                    print 'turn right'
                    twist.angular.z = 1.0
                elif event.key == pygame.K_ESCAPE:
                    sys.exit()

                twist_pub.publish(twist)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
