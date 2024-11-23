from typing import List, Tuple, Optional
import rospy
from geometry_msgs.msg import Quaternion
from pyquaternion import Quaternion as PyQuaternion
import math

class RobotCommandParser:
    def __init__(self):
        self.commands = {
            "move": self._handle_move,
            "grab": self._handle_grab,
            "release": self._handle_release,
            "home": self._handle_home,
            "rotate": self._handle_rotate
        }
        
    def parse_command(self, command_text: str) -> Tuple[str, dict]:
        """Parse text command into function name and parameters"""
        parts = command_text.lower().split()
        if not parts:
            raise ValueError("Empty command")
            
        command = parts[0]
        if command not in self.commands:
            raise ValueError(f"Unknown command: {command}")
            
        return self.commands[command](parts[1:])
        
    def _handle_move(self, args: List[str]) -> Tuple[str, dict]:
        """Handle move commands like: move x:0.5 y:0.3 z:0.7"""
        params = {}
        for arg in args:
            if ':' not in arg:
                continue
            axis, value = arg.split(':')
            if axis in ['x', 'y', 'z']:
                params[axis] = float(value)
        
        return "move", params
        
    def _handle_grab(self, args: List[str]) -> Tuple[str, dict]:
        """Handle grab command"""
        return "grab", {}
        
    def _handle_release(self, args: List[str]) -> Tuple[str, dict]:
        """Handle release command"""
        return "release", {}
        
    def _handle_home(self, args: List[str]) -> Tuple[str, dict]:
        """Handle home command"""
        return "home", {}
        
    def _handle_rotate(self, args: List[str]) -> Tuple[str, dict]:
        """Handle rotate commands like: rotate roll:90 pitch:45 yaw:180"""
        params = {}
        for arg in args:
            if ':' not in arg:
                continue
            axis, value = arg.split(':')
            if axis in ['roll', 'pitch', 'yaw']:
                params[axis] = math.radians(float(value))
        
        return "rotate", params
