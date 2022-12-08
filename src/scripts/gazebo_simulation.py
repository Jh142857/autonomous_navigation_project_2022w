import rospy
import numpy as np


from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

def create_model_state(x, y, z, angle):
    # the rotation of the angle is in (0, 0, 1) direction
    # gazebo_msgs/ModelState类型的消息
    model_state = ModelState()
    model_state.model_name = 'jackal'
    model_state.pose.position.x = x
    model_state.pose.position.y = y
    model_state.pose.position.z = z
    # 四元数初始化方向
    model_state.pose.orientation = Quaternion(0, 0, np.sin(angle/2.), np.cos(angle/2.))
    model_state.reference_frame = "world"

    return model_state


class GazeboSimulation():

    def __init__(self, init_position = [0, 0, 0]):
        # 不同的服务通信，前面是话题，后面是srv类型
        self._pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self._unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self._reset = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self._model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # 初始化输入x y theta
        self._init_model_state = create_model_state(init_position[0], init_position[1], 0, init_position[2])
        
        self.collision_count = 0
        # 接收/collision话题的数据，每次接收到到数据，collision_count++；
        self._collision_sub = rospy.Subscriber("/collision", Bool, self.collision_monitor)
        # 发布Twist类型的速度
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
    def collision_monitor(self, msg):
        if msg.data:
            self.collision_count += 1
    
    def get_hard_collision(self):
        # hard collision count since last call
        collided = self.collision_count > 0
        self.collision_count = 0
        return collided

    # 暂停服务
    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self._pause()
        except rospy.ServiceException:
            print ("/gazebo/pause_physics service call failed")

    # 取消暂停服务
    def unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self._unpause()
        except rospy.ServiceException:
            print ("/gazebo/unpause_physics service call failed")

    # 将机器人放回原点
    def reset(self):
        """
        /gazebo/reset_world or /gazebo/reset_simulation will
        destroy the world setting, here we used set model state
        to put the model back to the origin
        """
        rospy.wait_for_service("/gazebo/set_model_state")
        try:
            self._reset(self._init_model_state)
        except (rospy.ServiceException):
            rospy.logwarn("/gazebo/set_model_state service call failed")

    # 发布速度信息，主要是前进的线速度和角速度
    def pub_cmd_vel(self, data):
        cmd_vel_value = Twist()
        cmd_vel_value.linear.x, cmd_vel_value.angular.z = data[0], data[1]
        self._pub_cmd_vel.publish(cmd_vel_value)
                
    # 返回雷达数据
    def get_laser_scan(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('front/scan', LaserScan, timeout=5)
            except:
                pass
        return data

    # 获取机器人当前的位姿、方向信息
    def get_model_state(self):
        rospy.wait_for_service("/gazebo/get_model_state")
        try:
            # 输入是请求参数，输出是响应参数
            return self._model_state('jackal', 'world')
        except (rospy.ServiceException):
            rospy.logwarn("/gazebo/get_model_state service call failed")

    # 使机器人设置回初始位置
    def reset_init_model_state(self, init_position = [0, 0, 0]):
        """Overwrite the initial model state
        Args:
            init_position (list, optional): initial model state in x, y, z. Defaults to [0, 0, 0].
        """
        self._init_model_state = create_model_state(init_position[0],init_position[1],0,init_position[2])

    # def pub_goal_point(self, )