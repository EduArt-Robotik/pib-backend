from typing import Iterable, Tuple
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from datatypes.msg import MotorSettings, SolidStateRelayState
from datatypes.srv import ApplyMotorSettings, ApplyJointTrajectory, GetJointPosition
from datatypes.action import MoveToPose
from pib_api_client import motor_client, pose_client
from pib_motors.bricklet import ipcon, connected_enumerate, solid_state_relay_bricklet
from pib_motors.motor import name_to_motors, motors
from pib_motors.startup_pose_executor import StartupPoseExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def motor_settings_ros_to_dto(ms: MotorSettings):
    return {
        "name": ms.motor_name,
        "turnedOn": ms.turned_on,
        "pulseWidthMin": ms.pulse_width_min,
        "pulseWidthMax": ms.pulse_width_max,
        "rotationRangeMin": ms.rotation_range_min,
        "rotationRangeMax": ms.rotation_range_max,
        "velocity": ms.velocity,
        "acceleration": ms.acceleration,
        "deceleration": ms.deceleration,
        "period": ms.period,
        "visible": ms.visible,
        "invert": ms.invert,
    }


def as_motor_positions(jt: JointTrajectory) -> Iterable[Tuple[str, float]]:
    """
    Unpacks a JointTrajectory message into (motor_name, position) pairs.
    Handles standard single-waypoint messages and non-standard multi-point layouts.
    """
    if not jt.points:
        return []

    # Standard format: 1 point containing all N joint positions
    if len(jt.points) == 1:
        return zip(jt.joint_names, jt.points[0].positions)

    # Non-standard fallback: N points each containing 1 position value
    if len(jt.points) == len(jt.joint_names):
        positions = [p.positions[0] for p in jt.points if p.positions]
        return zip(jt.joint_names, positions)

    return []


def as_joint_trajectory(motor_name: str, position: int) -> JointTrajectory:
    """converts a motorname and position into a simple jt-message"""
    jt = JointTrajectory()
    jt.joint_names = [motor_name]
    point = JointTrajectoryPoint()
    point.positions.append(position)
    jt.points = [point]
    return jt


class MotorControl(Node):

    def __init__(self):

        super().__init__("motor_control")

        # Toggle Devmode
        self.declare_parameter("dev", False)
        self.dev = self.get_parameter("dev").value

        # Service for JointTrajectory
        self.srv = self.create_service(
            ApplyJointTrajectory, "apply_joint_trajectory", self.apply_joint_trajectory
        )

        # Publisher for JointTrajectory
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, "joint_trajectory", 10
        )

        # Service for MotorSettings
        self.srv = self.create_service(
            ApplyMotorSettings, "apply_motor_settings", self.apply_motor_settings
        )

        # Service for Getting Joint Position
        self.srv_get_position = self.create_service(
            GetJointPosition, "get_joint_position", self.get_joint_position
        )

        # Publisher for MotorSettings
        self.motor_settings_publisher = self.create_publisher(
            MotorSettings, "motor_settings", 10
        )

        # Action server for pose
        self.pose_action_server = ActionServer(
            self,
            MoveToPose,
            "move_to_pose",
            goal_callback=self.move_to_pose_goal,
            execute_callback=self.move_to_pose_execute,
            cancel_callback=self.move_to_pose_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

        self._startup_done = False

        # load motor-settings if not in dev mode
        if not self.dev:
            for motor in motors:
                if motor.check_if_motor_is_connected():
                    successful, motor_settings_dto = motor_client.get_motor_settings(
                        motor.name
                    )
                    if successful:
                        if not self._startup_done:
                            motor_settings_dto["turnedOn"] = False
                        motor.apply_settings(motor_settings_dto)

        # Log that initialization is complete
        self.get_logger().info("Now Running MOTOR_CONTROL")

        # Register and trigger enumeration of available bricklets to detect connected devices
        ipcon.register_callback(ipcon.CALLBACK_ENUMERATE, connected_enumerate)
        ipcon.enumerate()

        self.ssr_subscriber = self.create_subscription(
            SolidStateRelayState,
            "solid_state_relay_state",
            self.on_ssr_state_change,
            10,
        )

        self.startup_pose_executor = StartupPoseExecutor(self, motors=motors)

        # No Startup pose for the Chatbot Robot. Button 1 is configured to set a neutral pose.
        # if solid_state_relay_bricklet is None:
        #     self.get_logger().info(
        #         "No SSR configured. Executing startup pose immediately."
        #     )
        #     self._startup_done = True
        #     self._execute_startup_pose()

    def _execute_startup_pose(self):
        try:
            success = self.startup_pose_executor.execute()
            if success:
                self.get_logger().info("Startup pose execution completed successfully.")
            else:
                self.get_logger().warn(
                    "Startup pose execution completed with warnings."
                )
        except Exception as e:
            self.get_logger().error(
                f"Unexpected error while applying startup pose: {str(e)}"
            )

    def move_to_pose_goal(self, goal_request):
        pose_name = goal_request.pose_name
        successful, pose = pose_client.get_pose_by_name(pose_name)

        if not successful or pose is None:
            self.get_logger().warn(f"Could not find pose '{pose_name}'")
            return GoalResponse.REJECT

        successful, motor_positions_dict = pose_client.get_motor_positions_of_pose(
            pose["poseId"]
        )
        if not successful or not motor_positions_dict:
            self.get_logger().warn(
                f"Could not load motor positions for '{pose_name}'"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def move_to_pose_cancel(self, goal_handle):
        self.get_logger().info("Received request to cancel goal.")
        return CancelResponse.ACCEPT

    async def move_to_pose_execute(self, goal_handle):
        pose_name = goal_handle.request.pose_name

        successful, pose = pose_client.get_pose_by_name(pose_name)
        successful, motor_positions_dict = pose_client.get_motor_positions_of_pose(
            pose["poseId"]
        )

        # Convert dictionary from API into JointTrajectory ROS message
        goal_jt = self.pose_dict_to_joint_trajectory(motor_positions_dict)

        feedback_msg = MoveToPose.Feedback()
        result_msg = MoveToPose.Result()

        feedback_msg.goal_joint_trajectory = goal_jt
        goal_handle.publish_feedback(feedback_msg)

        # Execute motor trajectory
        trajectory_success = self._apply_joint_trajectory_msg(goal_jt)

        if trajectory_success:
            goal_handle.succeed()
            result_msg.success = True
            result_msg.error_code = MoveToPose.Result.SUCCESS
        else:
            goal_handle.abort()
            result_msg.success = False
            result_msg.error_code = MoveToPose.Result.TRAJECTORY_FAILED

        return result_msg

    def pose_dict_to_joint_trajectory(self, motor_positions_data: dict[str, any]) -> JointTrajectory:
        jt = JointTrajectory()
        joint_names = []
        positions = []

        if isinstance(motor_positions_data, dict):
            # Case B: API response wrapped under a key containing a list of objects
            if "motorPositions" in motor_positions_data:
                for item in motor_positions_data["motorPositions"]:
                    joint_names.append(item["motorName"])
                    positions.append(float(item["position"]))
            elif "positions" in motor_positions_data:
                for item in motor_positions_data["positions"]:
                    joint_names.append(item["motorName"])
                    positions.append(float(item["position"]))
            # Case A: Key-value mapping {"joint_name": position_value}
            else:
                joint_names = list(motor_positions_data.keys())
                positions = [float(pos) for pos in motor_positions_data.values()]

        jt.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        jt.points = [point]

        return jt
        
    def _apply_joint_trajectory_msg(self, jt: JointTrajectory) -> bool:
        overall_success = True
        try:
            self.get_logger().debug(f"Applying joint-trajectory: {jt} with {len(jt.points)} points for {len(jt.joint_names)} joints.")
            for motor_name, position in as_motor_positions(jt):
                for motor in name_to_motors[motor_name]:
                    self.get_logger().info(
                        f"setting position of {motor.name} to {position}"
                    )
                    successful = motor.set_position(position)
                    self.get_logger().info(
                        f"setting position {'succeeded' if successful else 'failed'}."
                    )
                    overall_success &= successful
                    self.joint_trajectory_publisher.publish(
                        as_joint_trajectory(motor.name, position)
                    )
        except Exception as e:
            overall_success = False
            self.get_logger().error(f"error while applying joint-trajectory: {str(e)}")
        return overall_success

    def on_ssr_state_change(self, msg: SolidStateRelayState):
        # No Startup pose for the Chatbot Robot. Button 1 is configured to set a neutral pose.
        #if not msg.turned_on or self._startup_done:
        #    return
        #self._startup_done = True
        #self._execute_startup_pose()
        pass

    def apply_motor_settings(
        self, request: ApplyMotorSettings.Request, response: ApplyMotorSettings.Response
    ) -> ApplyMotorSettings.Response:

        response.settings_applied = True
        response.settings_persisted = True

        motor_settings_ros = request.motor_settings
        motor_settings_dto = motor_settings_ros_to_dto(motor_settings_ros)

        try:
            motors = name_to_motors[request.motor_settings.motor_name]
            for motor in motors:
                motor_settings_dto["name"] = motor.name
                motor_settings_ros.motor_name = motor.name
                applied = motor.apply_settings(motor_settings_dto)
                response.settings_applied &= applied
                if applied or self.dev:
                    persisted, _ = motor_client.update_motor_settings(
                        motor.name, motor_settings_dto
                    )
                    response.settings_persisted &= persisted
                    self.motor_settings_publisher.publish(motor_settings_ros)
                self.get_logger().info(f"updated motor: {str(motor)}")

        except Exception as e:
            response.settings_applied = False
            response.settings_persisted = False
            self.get_logger().warn(
                f"Error while processing motor-settings-message: {str(e)}"
            )

        return response

    def apply_joint_trajectory(
        self,
        request: ApplyJointTrajectory.Request,
        response: ApplyJointTrajectory.Response,
    ) -> ApplyJointTrajectory.Response:
        jt = request.joint_trajectory
        response.successful = self._apply_joint_trajectory_msg(jt)
        return response

    def get_joint_position(
        self,
        request: GetJointPosition.Request,
        response: GetJointPosition.Response,
    ) -> GetJointPosition.Response:
        joint_name = request.joint_name
        response.successful = True
        try:
            if joint_name not in name_to_motors:
                response.successful = False
                response.message = f"unknown joint name '{joint_name}'"
                return response

            motors_for_joint = name_to_motors[joint_name]
            motor = motors_for_joint[0]
            response.position = motor.get_position()

        except Exception as e:
            response.successful = False
            response.message = str(e)
            self.get_logger().error(f"error getting position: {str(e)}")
        return response


def main(args=None):

    rclpy.init(args=args)
    motor_control = MotorControl()
    
    # Use MultiThreadedExecutor so callbacks process smoothly during execution
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(motor_control)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        motor_control.destroy_node()
        rclpy.shutdown()
        ipcon.disconnect()


if __name__ == "__main__":
    main()
