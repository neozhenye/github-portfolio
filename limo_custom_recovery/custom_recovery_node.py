#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray, GoalID
from std_srvs.srv import Empty

class ReverseRecovery:
    def __init__(self):
        rospy.init_node('reverse_recovery_node')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback)

        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmap_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        self.last_goal = None
        self.current_status = None
        self.recovering = False

        rospy.loginfo("ReverseRecovery node with infinite clear+resend logic ready.")
        rospy.spin()

    def goal_callback(self, msg):
        self.last_goal = msg
        rospy.loginfo("Stored new navigation goal.")

    def status_callback(self, msg):
        if not msg.status_list:
            return

        self.current_status = msg.status_list[-1].status

        if self.current_status == 4 and not self.recovering:  # ABORTED
            rospy.logwarn("MoveBase ABORTED! Attempting repeated clear + resend.")
            self.attempt_clear_and_resend()

    def attempt_clear_and_resend(self):
        self.recovering = True

        # Step 1: Cancel the current goal
        rospy.loginfo("Cancelling current goal.")
        self.cancel_pub.publish(GoalID())
        rospy.sleep(0.5)

        # Step 2: Clear costmaps repeatedly until no longer ABORTED
        max_attempts = 3  # safety limit
        attempts = 0
        while self.current_status == 4 and attempts < max_attempts and not rospy.is_shutdown():
            try:
                rospy.loginfo("Clearing costmaps... (attempt %d)", attempts + 1)
                self.clear_costmap_srv()
            except rospy.ServiceException as e:
                rospy.logerr("Failed to call clear_costmaps: %s", e)

            rospy.sleep(1.0)
            attempts += 1

        if self.current_status != 4:
            rospy.loginfo("Robot recovered after costmap clearing. Resending goal.")
            if self.last_goal:
                self.goal_pub.publish(self.last_goal)
            else:
                rospy.logwarn("No last goal to resend.")
            self.recovering = False
        else:
            rospy.logwarn("Still aborted after clearing. Executing reverse recovery.")
            self.run_reverse_recovery()

    def run_reverse_recovery(self):
        # Step 1: Reverse
        twist = Twist()
        twist.linear.x = -0.06
        duration = 0.5
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < rospy.Duration(duration):
            self.cmd_pub.publish(twist)
            rate.sleep()

        # Step 2: Stop
        twist = Twist()
        self.cmd_pub.publish(twist)
        rospy.sleep(0.5)

        #Step 3: Spin in place (rotate 360 slowly)
        twist.angular.z = 0.5  # positive = counter-clockwise
        spin_time = 2*3.14 / twist.angular.z  

        rospy.loginfo("Spinning in place to help localization...")
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(spin_time):
            self.cmd_pub.publish(twist)
            rate.sleep()

        # Stop
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.5)

        # Step 4: Clear costmaps again
        try:
            self.clear_costmap_srv()
            rospy.loginfo("Final costmap clear after reverse + spin.")
        except rospy.ServiceException as e:
            rospy.logerr("Final costmap clear failed: %s", e)

        # Step 5: Resend goal
        if self.last_goal:
            rospy.loginfo("Resending goal after reverse and spin.")
            self.goal_pub.publish(self.last_goal)
        else:
            rospy.logwarn("No goal available to resend.")

        self.recovering = False

if __name__ == '__main__':
    try:
        ReverseRecovery()
    except rospy.ROSInterruptException:
        pass
