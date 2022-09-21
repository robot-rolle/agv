import rospy,math
from costmap_converter.msg import ObstacleArrayMsg,ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32
def publish_obstacle_msg():
    pub = rospy.Publisher('/test_optim_node/obstacles',ObstacleArrayMsg,queue_size=1)
    rospy.init_node("test_obstacle_msg")

    obstacle_msg = ObstacleArrayMsg()
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id ='odom'

    obstacle_msg.obstacles.append(ObstacleMsg())
    obstacle_msg.obstacles[0].id = 0
    obstacle_msg.obstacles[0].polygon.points = [Point32()]
