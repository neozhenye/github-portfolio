#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# Define center waypoint (adjust as needed for your map)
center_waypoint = [(3.057, 1.117, 0.0), (0.0, 0.0, 0.115, 0.993)]  # Example

# Team waypoints (numbered 1 to 8)
waypoints_with_team = {
    1: [(2.028, -0.643, 0.0), (0.0, 0.0, 0.146, 0.989)],
    2: [(1.712, 0.634, 0.0), (0.0, 0.0, 0.147, 0.989)],
    3: [(1.392, 2.041, 0.0), (0.0, 0.0, 0.086, 0.996)],
    4: [(2.677, 2.457, 0.0), (0.0, 0.0, -0.645, 0.764)],
    5: [(4.462, 3.189, 0.0), (0.0, 0.0, -0.443, 0.897)],
    6: [(4.579, 1.328, 0.0), (0.0, 0.0, -0.636, 0.772)],
    7: [(4.871, 0.158, 0.0), (0.0, 0.0, 0.792, 0.611)],
    8: [(3.408, -0.186, 0.0), (0.0, 0.0, 0.77, 0.638)]
}

# Define which teams require passing through center *when going from one to another*
center_required_teams = [2, 4, 6, 8]

# Sequence of team numbers to visit
teams_to_visit = [1, 2, 3, 4, 5, 6, 7, 8]  # You can customize this order


def goal_pose(pose):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = pose[0][0]
    goal.target_pose.pose.position.y = pose[0][1]
    goal.target_pose.pose.position.z = pose[0][2]

    goal.target_pose.pose.orientation.x = pose[1][0]
    goal.target_pose.pose.orientation.y = pose[1][1]
    goal.target_pose.pose.orientation.z = pose[1][2]
    goal.target_pose.pose.orientation.w = pose[1][3]

    return goal


if __name__ == '__main__':
    rospy.init_node('selective_center_navigator')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base!")

    MAX_RETRIES = 20

    for i in range(len(teams_to_visit) - 1):
        from_team = teams_to_visit[i]
        to_team = teams_to_visit[i + 1]

        # Start building the list of goals
        goal_sequence = []

        # Add center if both from and to teams are in [2,4,6,8]
        if from_team in center_required_teams and to_team in center_required_teams:
            goal_sequence.append(("Center", goal_pose(center_waypoint)))

        # Always go to the destination team
        goal_sequence.append(("Team {to_team}", goal_pose(waypoints_with_team[to_team])))

        for label, goal in goal_sequence:
            success = False
            for attempt in range(MAX_RETRIES):
                rospy.loginfo("Sending goal")
                client.send_goal(goal)
                client.wait_for_result()

                state = client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Reached successfully.")
                    rospy.sleep(3.0)
                    success = True
                    break
                else:
                    rospy.logwarn("Attempt {attempt + 1} to reach {label} failed.")

            if not success:
                rospy.logerr("Failed to reach after max attempts. Aborting.")
                exit(1)

    rospy.loginfo("Completed all team navigation steps.")
