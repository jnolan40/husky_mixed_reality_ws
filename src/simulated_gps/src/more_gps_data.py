from ubxtranslator.msg import hpposllh, relpos2D
from geometry_msgs.msg import Twist
import rospy
class message_combiner:
    def __init__(self) :
        self.msg = Twist()
        self.Pub = rospy.Publisher('/UBX/Combined', Twist, queue_size=10)

    def run(self) :
        rospy.init_node('message_combiner_node')

        rospy.Subscriber('/UBX/relpos2D', relpos2D, self.hdg_cb)
        rospy.Subscriber('/UBX/hpposllh', hpposllh, self.llh_cb)

        rospy.spin()

    def hdg_cb(self, msg):
    
        # Convert to ENU
        theta = 90 - msg.pos.theta
        while theta > 180:
            theta -= 360
        while theta < -180:
            theta += 360
        self.msg.linear.z = theta
        
    def llh_cb(self, msg) :
        self.msg.linear.x = msg.llh.lat 
        self.msg.linear.y = msg.llh.lon
        self.msg.angular.x = msg.err.lat
        self.msg.angular.y = msg.err.lon 

        self.Pub.publish(self.msg) 

if __name__ == '__main__' :

    mc = message_combiner()
    mc.run()
