#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from competition_state_service.srv import GetState, GetStateResponse, SetState, SetStateResponse

class StateControlNode:
    def __init__(self):
        rospy.init_node('state_control_node')

        # Define allowed states
        self.allowed_states = ['Idle', 'Search', 'Social', 'Assess', 'Help']

        # Publisher for current state
        self.state_pub = rospy.Publisher('/competition_state', String, queue_size=10)

        # Internal state initialized to 'Idle'
        self.current_state = "Idle"
        rospy.sleep(0.5)  # Short delay to ensure publisher is connected before first publish
        self.state_pub.publish(String(self.current_state))
        rospy.loginfo(f"[StateControl] Default state set to: {self.current_state}")

        # Services
        self.set_state_srv = rospy.Service('/competition_state/set_state', SetState, self.handle_set_state)
        self.get_state_srv = rospy.Service('/competition_state/get_state', GetState, self.handle_get_state)

        rospy.loginfo("StateControlNode is ready.")
        rospy.spin()

    def handle_get_state(self, req):
        return GetStateResponse(current_state=self.current_state)

    def handle_set_state(self, req):
        new_state = req.new_state.strip()

        if not new_state:
            return SetStateResponse(success=False, message="Empty state is invalid.")

        if new_state not in self.allowed_states:
            allowed = ", ".join(self.allowed_states)
            return SetStateResponse(success=False, message=f"Invalid state. Allowed states: {allowed}")

        rospy.loginfo(f"[StateControl] Changing state to: {new_state}")
        self.current_state = new_state
        self.state_pub.publish(String(self.current_state))
        return SetStateResponse(success=True, message=f"State set to {new_state}")

if __name__ == '__main__':
    try:
        StateControlNode()
    except rospy.ROSInterruptException:
        pass

