import rospy
from std_msgs.msg import Int32MultiArray

class ObstacleDetector:

    def __init__(self, topic):
        self.distance = [[300, 300, 300, 300, 300]]
        rospy.Subscriber(topic, Int32MultiArray, self.read_distance)

    def read_distance(self, data):
        self.distance = self.distance[1:] + [data.data[1]]

    def get_distance(self):
        total = 0
        divisor = 0
        for i, distance in enumerate(self.distance):
            total += distance * (i + 1)
            divisor += i + 1
        return total // divisor
