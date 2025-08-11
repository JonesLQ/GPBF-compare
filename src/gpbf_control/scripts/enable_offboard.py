#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
import time

# 全局变量
current_state = State()

# 回调函数：更新当前状态
def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('offboard_enabler')
    
    # 创建订阅者、服务客户端和发布者
    state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')
    
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    
    # 设置循环频率
    rate = rospy.Rate(10)  # 10 Hz
    
    # 等待飞控连接
    rospy.loginfo("等待飞控连接...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    
    rospy.loginfo("飞控已连接！")
    
    # 发送初始姿态
    pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2  # 起飞高度2米
    
    # 发送一些初始姿态消息 (PX4需要在切换到OFFBOARD前收到至少几个姿态消息)
    rospy.loginfo("发送初始姿态消息...")
    for i in range(100):
        pose.header.stamp = rospy.Time.now()
        pose_pub.publish(pose)
        rate.sleep()
    
    # 解锁并切换到OFFBOARD模式
    rospy.loginfo("请求切换到OFFBOARD模式并解锁...")
    last_request = rospy.Time.now()
    
    while not rospy.is_shutdown():
        # 请求切换到OFFBOARD模式
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            response = set_mode_client(custom_mode="OFFBOARD")
            if response.mode_sent:
                rospy.loginfo("OFFBOARD模式已启用")
            else:
                rospy.logwarn("无法切换到OFFBOARD模式")
            last_request = rospy.Time.now()
        # 请求解锁
        elif not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            response = arming_client(True)
            if response.success:
                rospy.loginfo("飞机已解锁")
            else:
                rospy.logwarn("解锁失败")
            last_request = rospy.Time.now()
        # 如果已经解锁并在OFFBOARD模式，显示状态
        elif current_state.mode == "OFFBOARD" and current_state.armed:
            rospy.loginfo("无人机已解锁并在OFFBOARD模式 - 开始跟踪轨迹")
            time.sleep(2)  # 等待2秒
            rospy.loginfo("控制现在交给GP-PBF控制器，按Ctrl+C退出")
            break
        
        rate.sleep()
    
    # 保持节点运行
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("用户中断，退出")