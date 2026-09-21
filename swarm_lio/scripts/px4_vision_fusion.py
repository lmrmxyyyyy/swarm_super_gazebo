#!/usr/bin/env python3

"""Enable PX4 external-vision fusion after Swarm-LIO2 becomes ready."""

import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ParamValue, State
from mavros_msgs.srv import ParamSet, ParamSetRequest


class VisionFusionConfigurator:
    def __init__(self):
        self.mavros_ns = rospy.get_param("~mavros_ns").rstrip("/")
        self.vision_topic = rospy.get_param("~vision_topic")
        self.required_samples = rospy.get_param("~required_samples", 20)
        self.connected = False
        self.valid_pose_samples = 0

        rospy.Subscriber(
            self.mavros_ns + "/state", State, self._state_callback, queue_size=1
        )
        rospy.Subscriber(
            self.vision_topic, PoseStamped, self._pose_callback, queue_size=10
        )

    def _state_callback(self, message):
        self.connected = message.connected

    def _pose_callback(self, message):
        values = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        if all(math.isfinite(value) for value in values):
            self.valid_pose_samples = min(
                self.valid_pose_samples + 1, self.required_samples
            )
        else:
            self.valid_pose_samples = 0

    def wait_until_ready(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.connected and self.valid_pose_samples >= self.required_samples:
                return True
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                return False
        return False

    def set_integer_parameter(self, name, value):
        service_name = self.mavros_ns + "/param/set"
        rospy.wait_for_service(service_name, timeout=15.0)
        service = rospy.ServiceProxy(service_name, ParamSet)
        request = ParamSetRequest(
            param_id=name, value=ParamValue(integer=value, real=0.0)
        )

        for attempt in range(1, 6):
            try:
                response = service(request)
                if response.success and response.value.integer == value:
                    rospy.loginfo("Set %s=%d through %s", name, value, service_name)
                    return True
                rospy.logwarn(
                    "PX4 rejected %s=%d on attempt %d", name, value, attempt
                )
            except rospy.ServiceException as error:
                rospy.logwarn("Failed to set %s: %s", name, error)
            rospy.sleep(1.0)
        return False


def main():
    rospy.init_node("px4_vision_fusion")
    configurator = VisionFusionConfigurator()
    rospy.loginfo(
        "Waiting for MAVROS and %d valid poses on %s",
        configurator.required_samples,
        configurator.vision_topic,
    )
    if not configurator.wait_until_ready():
        return

    aid_ok = configurator.set_integer_parameter("EKF2_AID_MASK", 24)
    height_ok = configurator.set_integer_parameter("EKF2_HGT_MODE", 3)
    if aid_ok and height_ok:
        rospy.loginfo("PX4 external-vision fusion configured for %s", configurator.mavros_ns)
    else:
        rospy.logerr("Could not fully configure PX4 vision fusion for %s", configurator.mavros_ns)


if __name__ == "__main__":
    main()
