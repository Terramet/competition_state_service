#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from competition_state_service.srv import GetState, GetStateResponse, SetState, SetStateResponse

class StateControlNode:
    def __init__(self):
        rospy.init_node('state_control_node')

        # Publisher for current state
        self.state_pub = rospy.Publisher('/competition_state', String, queue_size=10)

        # Internal state initialized to 'Idle'
        self.current_state = "Idle"
        self.state_pub.publish(String(self.current_state))
        rospy.loginfo(f"[StateControl] Default state set to: {self.current_state}")

        # Create the service for setting state
        self.set_state_srv = rospy.Service('/competition_state/set_state', SetState, self.handle_set_state)

        self.service = rospy.Service('/competition_state/get_state', GetState, self.handle_get_state)
        rospy.loginfo("StateServiceNode is ready.")
        rospy.spin()

    def handle_get_state(self, req):
        current = self.current_state if self.current_state else ""
        return GetStateResponse(current_state=current)

    def handle_set_state(self, req):
        new_state = req.new_state.strip()
        if not new_state:
            return SetStateResponse(success=False, message="Empty state is invalid.")

        rospy.loginfo(f"[StateControl] Received new state: {new_state}")
        self.current_state = new_state
        self.state_pub.publish(String(self.current_state))
        return SetStateResponse(success=True, message=f"State set to {new_state}")

if __name__ == '__main__':
    try:
        StateControlNode()
    except rospy.ROSInterruptException:
        pass

