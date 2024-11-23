#!/usr/bin/env python3

import os
import rospy
import actionlib
import control_msgs.msg
from controller import ArmController
from pyquaternion import Quaternion as PyQuaternion
from openai import OpenAI
import math
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import *
from gazebo_ros_link_attacher.srv import Attach, SetStatic  # Add this line

DEFAULT_POS = (0.4, 0, 0.6)
DEFAULT_QUAT = PyQuaternion(axis=[0, 1, 0], angle=math.pi)
INTERLOCKING_OFFSET = 0.001  # Small offset for stacking Legos

class OpenAIRobotAgent:
    def __init__(self):
        # Initialize OpenAI client
        api_key = "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"
        if not api_key:
            raise ValueError("Please set OPENAI_API_KEY environment variable")
        self.client = OpenAI(api_key=api_key)
        
        # Initialize ROS node and controllers
        rospy.init_node("openai_robot_agent")
        self.controller = ArmController()
        
        # Initialize gripper client
        self.gripper_client = actionlib.SimpleActionClient(
            "/gripper_controller/gripper_cmd",
            control_msgs.msg.GripperCommandAction
        )
        print("Waiting for gripper controller...")
        self.gripper_client.wait_for_server()

        # Initialize services for link attacher
        self.setstatic_srv = rospy.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
        self.attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
        self.detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)

    def get_visible_legos(self):
        """Get list of detected Legos from vision system"""
        try:
            print("\n=== Starting Lego Detection ===")
            print("Waiting for lego detections on /lego_detections topic...")
            
            # Try vision system first with shorter timeout
            try:
                legos = rospy.wait_for_message("/lego_detections", ModelStates, timeout=2.0)
                print(f"Raw lego message: {legos}")  # Add this debug print
            except rospy.ROSException as e:
                print(f"Timeout waiting for /lego_detections: {e}")
                legos = None
                
            if legos and hasattr(legos, 'name') and hasattr(legos, 'pose') and len(legos.name) > 0:
                print(f"Successfully detected {len(legos.name)} legos")
                for name, pose in zip(legos.name, legos.pose):
                    print(f"Lego: {name}")
                    print(f"Position: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
                return [(name, pose) for name, pose in zip(legos.name, legos.pose)]
            else:
                print("Vision system returned empty or invalid message")
                
            # Fallback to Gazebo
            print("\n=== Falling back to Gazebo ===")
            print("Trying to get legos from /gazebo/model_states...")
            
            models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=2.0)
            print(f"Raw Gazebo message: {models}")  # Add this debug print
            print(f"Model names: {models.name}")    # Add this debug print
            
            legos = []
            for name, pose in zip(models.name, models.pose):
                # Make pattern matching more flexible
                if any(lego_type in name for lego_type in ["X1-Y", "X2-Y"]):
                    clean_name = name.replace("lego_", "").split("_", maxsplit=1)[0]
                    legos.append((clean_name, pose))
                    print(f"Found Lego in Gazebo: {clean_name}")
                    print(f"Position: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
            
            if legos:
                print(f"Successfully found {len(legos)} legos in Gazebo")
                return legos
            else:
                print("No legos found in Gazebo")
                return []
                
        except Exception as e:
            print(f"Error in get_visible_legos: {str(e)}")
            return []

    def pick_lego(self, lego_name, lego_pose):
        """Pick up a specific Lego piece"""
        try:
            # Get the full Gazebo model name
            models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=2.0)
            gazebo_model_name = None
            for name in models.name:
                if lego_name in name:
                    gazebo_model_name = name
                    break
                    
            if not gazebo_model_name:
                raise ValueError(f"Could not find Gazebo model for {lego_name}")

            print(f"Moving to position above {lego_name}")
            # Pre-position above Lego
            approach_height = 0.01  # Reduced approach height
            self.controller.move_to(
                lego_pose.position.x,
                lego_pose.position.y,
                lego_pose.position.z + approach_height,
                DEFAULT_QUAT
            )
            
            print("Opening gripper")
            # Open gripper before approaching
            self.open_gripper()
            rospy.sleep(0.5)
            
            print("Moving down to grasp")
            # Move down to grasp position
            self.controller.move_to(
                lego_pose.position.x,
                lego_pose.position.y,
                lego_pose.position.z + 0.002,  # Very close to Lego surface
                DEFAULT_QUAT,
                duration=2.0  # Slower movement
            )
            
            print("Closing gripper")
            # Close gripper
            self.close_gripper()
            rospy.sleep(1.0)  # Give more time for grip
            
            print("Attaching Lego to gripper")
            # Attach Lego to gripper
            req = Attach()
            req.model_name_1 = gazebo_model_name
            req.link_name_1 = "link"
            req.model_name_2 = "robot"
            req.link_name_2 = "wrist_3_link"
            self.attach_srv.call(req)
            
            print("Lifting Lego")
            # Lift up slowly
            self.controller.move_to(
                lego_pose.position.x,
                lego_pose.position.y,
                lego_pose.position.z + 0.1,
                DEFAULT_QUAT,
                duration=2.0
            )
            
        except Exception as e:
            print(f"Error in pick_lego: {str(e)}")

    def process_command(self, user_input: str) -> str:
        try:
            # Make input case-insensitive
            user_input = user_input.lower()
            
            # Check for Lego-related commands with more flexible matching
            if ("lego" in user_input or "block" in user_input) and \
               ("pick" in user_input or "grab" in user_input or "get" in user_input):
                # Get visible Legos
                legos = self.get_visible_legos()
                
                if not legos:
                    return "No Legos currently visible"
                    
                # Take the first visible Lego
                lego_name, lego_pose = legos[0]
                print(f"Attempting to pick up {lego_name}")
                self.pick_lego(lego_name, lego_pose)
                return f"Picked up {lego_name}"
            
            # For non-Lego commands, use OpenAI API
            completion = self.client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": """You are a robot control assistant for a UR5 robotic arm.
                    Available commands:
                    1. move_to(x, y, z): Move end-effector to absolute position
                    2. move(dx, dy, dz): Move end-effector relative to current position
                    3. set_gripper(value): Control gripper (0.8=open, 0.0=closed)
                    4. rotate(roll, pitch, yaw): Rotate end-effector in degrees

                    
                    Convert natural language to exact function calls."""},
                    {"role": "user", "content": user_input}
                ],
                temperature=0,
                max_tokens=100
            )
            
            command = completion.choices[0].message.content.strip()
            return self.execute_command(command)
            
        except Exception as e:
            return f"Error processing command: {str(e)}"

    def execute_command(self, command: str) -> str:
        try:
            # Create a safe local execution environment with allowed functions
            safe_env = {
                'move_to': self.controller.move_to,
                'move': self.controller.move,
                'set_gripper': self.set_gripper,
                'rotate': self.rotate,
                'get_visible_legos': self.get_visible_legos,
                'pick_lego': self.pick_lego,
                'DEFAULT_POS': DEFAULT_POS,
                'DEFAULT_QUAT': DEFAULT_QUAT
            }
            
            # If command is a string description, just return it
            if command.startswith("Picked up") or command.startswith("No Legos"):
                return command
                
            # Execute the command in the safe environment
            exec(command, {"__builtins__": {}}, safe_env)
            return f"Executed: {command}"
            
        except Exception as e:
            return f"Error executing command: {str(e)}"

    def set_gripper(self, value):
        """
        Control the gripper
        value: 0.0 (fully open) to 0.8 (fully closed)
        """
        try:
            goal = control_msgs.msg.GripperCommandGoal()
            # Invert the value since the gripper works in reverse
            goal.command.position = 0.8 - value  # This inverts the behavior
            goal.command.max_effort = -1  # Do not limit the effort
            self.gripper_client.send_goal_and_wait(goal, rospy.Duration(10))
            return self.gripper_client.get_result()
        except Exception as e:
            print(f"Error controlling gripper: {str(e)}")

    def open_gripper(self):
        """Open the gripper fully"""
        return self.set_gripper(0.0)  # 0.0 for fully open

    def close_gripper(self):
        """Close the gripper fully"""
        return self.set_gripper(0.8)  # 0.8 for fully closed

    def rotate(self, roll: float, pitch: float, yaw: float):
        """Rotate end-effector (angles in degrees)"""
        rotation = PyQuaternion(axis=[1, 0, 0], angle=math.radians(roll)) * \
                  PyQuaternion(axis=[0, 1, 0], angle=math.radians(pitch)) * \
                  PyQuaternion(axis=[0, 0, 1], angle=math.radians(yaw))
        
        current_pos = self.controller.gripper_pose[0]
        self.controller.move_to(*current_pos, rotation)

if __name__ == "__main__":
    try:
        agent = OpenAIRobotAgent()
        print("OpenAI Robot Agent ready. Enter commands in natural language:")
        print("Examples:")
        print("- 'move the arm 20 centimeters up'")
        print("- 'rotate the gripper 90 degrees clockwise'")
        print("- 'open the gripper fully'")
        
        while not rospy.is_shutdown():
            try:
                command = input("> ")
                if command.lower() in ["quit", "exit"]:
                    break
                result = agent.process_command(command)
                print(result)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {str(e)}")
                
    except rospy.ROSInterruptException:
        pass
