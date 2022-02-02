
from ubxtranslator.msg import hpposllh, relpos2D
from geometry_msgs.msg import Pose2D
import rospy
class message_combiner:
    def __init__(self) :
        self.msg = Pose2D(0, 0, 0)
        self.Pub = rospy.Publisher('/UBX/Combined', Pose2D, queue_size=10)

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
        self.msg.theta = theta
        
    def llh_cb(self, msg) :
        self.msg.x = msg.llh.lat 
        self.msg.y = msg.llh.lon 

        self.Pub.publish(self.msg) 

if __name__ == '__main__' :

    mc = message_combiner()
    mc.run()
