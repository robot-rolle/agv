#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control mbot!
---------------------------
Moving around:
       w   
   a        d
       s    

q/z : increase/decrease max speeds by 10%

e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""
#键值对应移动/转向方向
moveBindings = {
        'w':(1,0),
        # 'o':(1,-1),
        'a':(0,1),
        'd':(0,-1),
        # 'u':(1,1),
        's':(-1,0),
        # '.':(-1,1),
        # 'm':(-1,-1),
           }
#键值对应速度增量
speedBindings={
        'q':(1.1,1.1),
        'z':(0.9,0.9),
        # 'w':(1.1,1),
        # 'x':(0.9,1),
        # 'e':(1,1.1),
        # 'c':(1,0.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1) # time set
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 0.2  #0.2m/s
turn = 0.1 # 0.1 rad/s

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)#获取键值初始化，读取终端相关属性
    
    rospy.init_node('hgrobot_teleop')#创建ROS节点
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)#创建速度话题发布者，'~cmd_vel'='节点名/cmd_vel'

    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0

    x      = 0   #前进后退方向
    th     = 0   #转向/横向移动方向
    count  = 0   #键值不再范围计数
    target_speed = 0 #前进后退目标速度
    target_turn  = 0 #转向目标速度
    control_speed = 0 #前进后退实际控制速度
    control_turn  = 0 #转向实际控制速度

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            # 运动控制方向键（1：正方向，-1负方向）
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            # 速度修改键
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]  # 线速度增加0.1倍
                turn = turn * speedBindings[key][1]    # 角速度增加0.1倍
                count = 0
                print(vels(speed,turn))
                # if (status == 14):
                #     print(msg)
                # status = (status + 1) % 15
            # 停止键
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count >=2:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            # 目标速度=速度值*方向值
            target_speed = speed * x
            target_turn = turn * th

            # 速度限位，防止速度增减过快
            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.04 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.04 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            # 创建并发布twist消息
            twist = Twist()
            twist.linear.x = control_speed; 
            twist.linear.y = 0; 
            twist.linear.z = 0
            twist.angular.x = 0; 
            twist.angular.y = 0; 
            twist.angular.z = control_turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
