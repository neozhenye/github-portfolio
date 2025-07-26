#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# Define center waypoint (adjust as needed for your map)
center_waypoint = [(-1.356, 1.555, 0.0), (0.0, 0.0, 0.987, 0.162)]

# Team waypoints (numbered 1 to 8)
waypoints_with_team = {
    1: [(-1.4, -1.52, 0.0), (0.0, 0.0, 0.0, 1.0)],
    2: [(-1.48, -0.15, 0.0), (0.0, 0.0, 1.000, 0.0)],  # fixed "00" to "0.0"
    3: [(-1.4, 1.13, 0.0), (0.0, 0.0, 0.0, 0.999)],
    4: [(-0.191, 1.275, 0.0), (0.0, 0.0, 0.72, 0.69)],
    5: [(1.582, 1.53, 0.0), (0.0, 0.0, -0.565, 0.816)],
    6: [(1.45, 0.12, 0.0), (0.0, 0.0, -0.742, 0.68)],
    7: [(1.5, -1.2, 0.0), (0.0, 0.0, -0.68, 0.725)],
    8: [(0.01, -1.35, 0.0), (0.0, 0.0, -0.69, 0.721)]
}

# Define which teams require passing through center when transitioning between each other
center_required_teams = [2, 4, 6, 8]

# Sequence of team numbers to visit
teams_to_visit = [6, 7, 8]  # You can customize this order


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

    # Go to the first team directly
    first_team = teams_to_visit[0]
    rospy.loginfo("Going to first team: Team {}".format(first_team))
    goal = goal_pose(waypoints_with_team[first_team])

    success = False
    for attempt in range(MAX_RETRIES):
        rospy.loginfo("Sending goal to Team {}, attempt {}...".format(first_team, attempt + 1))
        client.send_goal(goal)
        client.wait_for_result()

        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached Team {} successfully.".format(first_team))
            rospy.sleep(3.0)
            success = True
            break
        else:
            rospy.logwarn("Attempt {} to reach Team {} failed.".format(attempt + 1, first_team))

    if not success:
        rospy.logerr("Failed to reach Team {} after {} attempts. Aborting.".format(first_team, MAX_RETRIES))
        exit(1)

    # Now go through the rest of the route
    for i in range(len(teams_to_visit) - 1):
        from_team = teams_to_visit[i]
        to_team = teams_to_visit[i + 1]

        goal_sequence = []

        # Always go to center if destination is team 8
        if to_team == 8:
            goal_sequence.append(("Center", goal_pose(center_waypoint)))

        # Otherwise, only go to center if both from and to are in the center_required_teams
        elif from_team in center_required_teams and to_team in center_required_teams:
            goal_sequence.append(("Center", goal_pose(center_waypoint)))

        # Always go to the destination team
        goal_sequence.append(("Team {}".format(to_team), goal_pose(waypoints_with_team[to_team])))

        for label, goal in goal_sequence:
            success = False
            for attempt in range(MAX_RETRIES):
                rospy.loginfo("Sending goal to {}, attempt {}...".format(label, attempt + 1))
                client.send_goal(goal)
                client.wait_for_result()

                state = client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Reached {} successfully.".format(label))
                    rospy.sleep(3.0)
                    success = True
                    break
                else:
                    rospy.logwarn("Attempt {} to reach {} failed.".format(attempt + 1, label))

            if not success:
                rospy.logerr("Failed to reach {} after {} attempts. Aborting.".format(label, MAX_RETRIES))
                exit(1)

    rospy.loginfo("Completed all team navigation steps.")
