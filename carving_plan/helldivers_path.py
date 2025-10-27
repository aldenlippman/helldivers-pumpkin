import sys
import math

# ROS Client Library
import rclpy
from rclpy.node import Node

# YAML Export Utility
from carving_plan.export_utility import ExportUtility

# ROS Data Types
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker

# Project Specific Data Types
from pumpkin_msgs.srv import PlanMotion

PUSH_IN = 0.04


# Node
class HelldiversPumpkinPath(Node):

    def __init__(self):
        super().__init__('alden_pumpkin_planner')

        # Client
        self.cli = self.create_client(PlanMotion, 'generate_motion_plan')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('generate_motion_plan service not available, waiting again...')

        #Publisher
        self.pose_array_publisher_ = self.create_publisher(PoseArray, 'plot_carve', 10)
        self.line_publisher_ = self.create_publisher(Marker, 'plot_carve_lines', 10)
        self.pose_array_publisher_1_ = self.create_publisher(PoseArray, 'plot_carve_1', 10)
        self.line_publisher_1_ = self.create_publisher(Marker, 'plot_carve_lines_1', 10)
        self.pose_array_publisher_2_ = self.create_publisher(PoseArray, 'plot_carve_2', 10)
        self.line_publisher_2_ = self.create_publisher(Marker, 'plot_carve_lines_2', 10)
        self.pose_array_publisher_3_ = self.create_publisher(PoseArray, 'plot_carve_3', 10)
        self.line_publisher_3_ = self.create_publisher(Marker, 'plot_carve_lines_3', 10)
        self.pose_array_publisher_4_ = self.create_publisher(PoseArray, 'plot_carve_4', 10)
        self.line_publisher_4_ = self.create_publisher(Marker, 'plot_carve_lines_4', 10)

        # Initialize Request Structure
        self.req = PlanMotion.Request()


    def send_request(self):
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def generate_tool_path_left(self):

        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is top left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.0, 0.0, 0.0))

        #Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.0, 0.0, PUSH_IN))

        #Draw out shape
        pumpkin_segment.poses.append(self.add_waypoint(0.0, 0.02, PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.02,0.02,PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.02,0.13,PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.0,0.13,PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.0,0.15,PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.07,0.15,PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.07,(0.15-0.0355),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.0615,(0.15-0.038),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.058,(0.15-0.0507),PUSH_IN)) 
        pumpkin_segment.poses.append(self.add_waypoint(0.05,(0.15-0.0475),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.0375,(0.15-0.0625),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.045,(0.15-0.07),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.045,(0.15-0.10265),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.07,(0.15-0.111),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.07,0.0,PUSH_IN))

        #Return to origin
        pumpkin_segment.poses.append(self.add_waypoint(0.0,0.0,PUSH_IN))

        #Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        self.get_logger().info("Completed Tool Path Left")
        return pumpkin_segment
    
    def generate_tool_path_right(self):

        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is top left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.15,0.0,0.0,0.0,0.0,0.0))

        #Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.15, 0.0, PUSH_IN, 0.0, 0.0, 0.0))

        #Draw out shape
        pumpkin_segment.poses.append(self.add_waypoint(0.15,(0.15-0.13),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.13,(0.15-0.13),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.13,(0.15-0.02),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.15,(0.15-0.02),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.15,(0.15),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.08,(0.15),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.08,(0.15-0.0355),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.0885,(0.15-0.038),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.092,(0.15-0.0507),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.1,(0.15-0.0475),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.1125,(0.15-0.0625),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.105,(0.15-0.07),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.105,(0.15-0.10265),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.08,(0.15-0.111),PUSH_IN))
        pumpkin_segment.poses.append(self.add_waypoint(0.08,0.0,PUSH_IN))

        #Return to origin
        pumpkin_segment.poses.append(self.add_waypoint(0.15,0.0,PUSH_IN,0.0,0.0,0.0))

        #Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.15,0.0,0.0,0.0,0.0,0.0))
        
        return pumpkin_segment
    
    def generate_tool_path_left_eye(self):

        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is top left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.0585,(0.15-0.0715), 0.0))

        #Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.0585,(0.15-0.0715),PUSH_IN))
        
        #Make circle
        for x in range(90, 450, 15):
            xcoord = 0.0585 + (math.sqrt(0.000025))*math.sin(x*(math.pi/180))
            ycoord = (0.15-0.0655) - (math.sqrt(0.000025))*math.cos(x*(math.pi/180))
            pumpkin_segment.poses.append(self.add_waypoint(xcoord,ycoord,PUSH_IN))

        #Return to starting position
        pumpkin_segment.poses.append(self.add_waypoint(0.0585,(0.15-0.0715), PUSH_IN))

        #Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.0585,(0.15-0.0715), 0.0))

        return pumpkin_segment
    
    def generate_tool_path_right_eye(self):

        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is top left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.0915,(0.15-0.0715), 0.0))

        #Push tool in
        pumpkin_segment.poses.append(self.add_waypoint(0.0915,(0.15-0.0715), PUSH_IN))
        
        #Make circle
        for x in range(90, 450, 15):
            xcoord = 0.0915 + (math.sqrt(0.000025))*math.sin(x*(math.pi/180))
            ycoord = (0.15-0.0655) - (math.sqrt(0.000025))*math.cos(x*(math.pi/180))
            pumpkin_segment.poses.append(self.add_waypoint(xcoord,ycoord,PUSH_IN))

        #Return to starting position
        pumpkin_segment.poses.append(self.add_waypoint(0.0915,(0.15-0.0715), PUSH_IN))

        #Pull tool out
        pumpkin_segment.poses.append(self.add_waypoint(0.0915,(0.15-0.0715), 0.0))

        return pumpkin_segment
    
    def generate_tool_path_mouth(self):

        pumpkin_segment = PoseArray()
        pumpkin_segment.header.frame_id = "pumpkin_face"

        #Origin is top left corner
        pumpkin_segment.poses.append(self.add_waypoint(0.0725,(0.15-0.05125), 0.0))

        pumpkin_segment.poses.append(self.add_waypoint(0.0725,(0.15-0.05125), PUSH_IN))

        pumpkin_segment.poses.append(self.add_waypoint(0.075,(0.15-0.055), PUSH_IN))

        pumpkin_segment.poses.append(self.add_waypoint(0.0775,(0.15-0.05125), PUSH_IN))

        return pumpkin_segment
    
    def plot_rviz(self, tool_path:PoseArray):

        # Generate lines
        tool_path_lines = Marker()
        tool_path_lines.header.frame_id = "pumpkin_face"
        tool_path_lines.type = Marker.LINE_STRIP
        tool_path_lines.action = Marker.ADD
        tool_path_lines.pose.orientation.w = 1.0
        tool_path_lines.scale.x = 0.002
        tool_path_lines.scale.y = 0.002
        tool_path_lines.scale.z = 0.002
        tool_path_lines.color.a = 0.8
        tool_path_lines.color.r = 1.0
        tool_path_lines.color.g = 1.0
        tool_path_lines.color.b = 1.0


        # Extract Points
        for pose in tool_path.poses:
            tool_path_lines.points.append(pose.position)

        return tool_path_lines

    def add_waypoint(self, px, py, pz, ox = 0.0, oy = 0.0, oz = 0.0):
        waypoint = Pose()
        waypoint.position.x = px
        waypoint.position.y = py
        waypoint.position.z = pz
        waypoint.orientation.x = ox
        waypoint.orientation.y = oy
        waypoint.orientation.z = oz
        waypoint.orientation.w = 1.0
        return waypoint

def main(args=None):


    rclpy.init(args=args)

    # Instantiate TPP Class
    minimal_client = HelldiversPumpkinPath()

    # Generate Pumpkin Toolpath
    tool_path_left = minimal_client.generate_tool_path_left()
    tool_path_right = minimal_client.generate_tool_path_right()
    tool_path_left_eye = minimal_client.generate_tool_path_left_eye()
    tool_path_right_eye = minimal_client.generate_tool_path_right_eye()
    tool_path_mouth = minimal_client.generate_tool_path_mouth()
    minimal_client.req.path = [tool_path_left_eye, tool_path_right_eye, tool_path_mouth, tool_path_left, tool_path_right]



    # Visualize toolpath
    minimal_client.pose_array_publisher_.publish(tool_path_left)
    minimal_client.line_publisher_.publish(minimal_client.plot_rviz(tool_path_left))
    minimal_client.pose_array_publisher_1_.publish(tool_path_right)
    minimal_client.line_publisher_1_.publish(minimal_client.plot_rviz(tool_path_right))
    minimal_client.pose_array_publisher_2_.publish(tool_path_left_eye)
    minimal_client.line_publisher_2_.publish(minimal_client.plot_rviz(tool_path_left_eye))
    minimal_client.pose_array_publisher_3_.publish(tool_path_right_eye)
    minimal_client.line_publisher_3_.publish(minimal_client.plot_rviz(tool_path_right_eye))
    minimal_client.pose_array_publisher_4_.publish(tool_path_mouth)
    minimal_client.line_publisher_4_.publish(minimal_client.plot_rviz(tool_path_mouth))
    # minimal_client.get_logger().info("Attempted to publish")
    
    # Write out toolpath to YAML
    ExportUtility("log").write_pose_array_list_to_yaml(minimal_client.req.path)
    

    # Send Motion Plan Request
    minimal_client.get_logger().info("Attempting to call motion planner!")
    motion_plan_response = minimal_client.send_request()


    # Print info about reponse
    # TODO
    # TODO
    # minimal_client.get_logger().info()

    trajectory_msg = motion_plan_response.trajectory

    print(trajectory_msg)
    # minimal_client.get_logger().info(trajectory_msg)


    rclpy.spin_once(minimal_client)
    ExportUtility("log").write_joint_trajectory_to_yaml(trajectory_msg)


if __name__ == '__main__':
    main()

