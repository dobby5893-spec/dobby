#!/usr/bin/env python3
"""Read-only navigation trial recorder; start before exploration. Ctrl-C writes overview.svg."""
import csv
import json
import math
import re
from datetime import datetime
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path as RosPath, Odometry
from action_msgs.msg import GoalStatusArray
from nav2_msgs.msg import BehaviorTreeLog
from rcl_interfaces.msg import Log
from tf2_ros import Buffer, TransformListener

class Recorder(Node):
    def __init__(self):
        super().__init__('navigation_trial_recorder')
        root=self.declare_parameter('record_root','/home/dobby/scan/test_records').value
        self.directory=Path(root)/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True)
        self.events=(self.directory/'events.jsonl').open('w',buffering=1)
        self.csvfile=(self.directory/'trajectory.csv').open('w',newline='',buffering=1)
        self.csv=csv.writer(self.csvfile)
        self.csv.writerow(['sim_time','goal_sequence','map_x','map_y','map_z','yaw','goal_x','goal_y','distance_to_goal'])
        self.goal=None;self.goal_sequence=0;self.samples=[];self.goals=[];self.last={};self.subs=[]
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.path=RosPath();self.path.header.frame_id='map'
        self.pub=self.create_publisher(RosPath,'/diagnostics/actual_path',1)
        for topic in ['/cmd_vel','/cmd_vel_nav','/scanplanner/cmd_vel']:
            self.subs.append(self.create_subscription(Twist,topic,lambda m,t=topic:self.message(t,m),20))
        for topic in ['/plan','/plan_smoothed','/scanplanner/initial_path','/local_plan']:
            self.subs.append(self.create_subscription(RosPath,topic,lambda m,t=topic:self.message(t,m),10))
        for topic in ['/odom','/fastlio/odom','/odom/ground_truth']:
            self.subs.append(self.create_subscription(Odometry,topic,lambda m,t=topic:self.message(t,m),qos_profile_sensor_data))
        for action in ['navigate_to_pose','follow_path','escape_move','backup','spin']:
            topic=f'/{action}/_action/status'
            qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subs.append(self.create_subscription(GoalStatusArray,topic,lambda m,t=topic:self.message(t,m),qos))
        self.subs.append(self.create_subscription(Log,'/rosout',self.log,100))
        self.subs.append(self.create_subscription(PoseStamped,'/explore/selected_frontier',self.selected,10))
        self.subs.append(self.create_subscription(BehaviorTreeLog,'/behavior_tree_log',lambda m:self.message('/behavior_tree_log',m),100))
        self.create_timer(0.2,self.sample)
        self.create_timer(5.0,self.graph)
        self.write('session',{'directory':str(self.directory),'note':'No motion commands published; goal_sequence comes from /explore/selected_frontier; action UUIDs recorded separately.'})
        print(f'Recording to {self.directory}',flush=True)
    def write(self,kind,data):
        self.events.write(json.dumps({'sim_time':self.get_clock().now().nanoseconds/1e9,'wall_time':datetime.now().isoformat(),'goal_sequence':self.goal_sequence,'kind':kind,'data':data},ensure_ascii=False)+'\n')
    def message(self,topic,msg,info=None):
        now=self.get_clock().now().nanoseconds/1e9
        if 'cmd_vel' in topic or 'odom' in topic:
            if now-self.last.get(topic,-1e9)<0.05:return
            self.last[topic]=now
        data=message_to_ordereddict(msg)
        if info is not None:data={'message':data,'publisher_gid':list(info.publisher_gid)}
        self.write(topic,data)
    def log(self,msg):
        if msg.name in ['frontier_explorer','controller_server','behavior_server','bt_navigator','smoother_server','planner_server']:
            self.write('log',{'node':msg.name,'level':msg.level,'text':msg.msg,'stamp':message_to_ordereddict(msg.stamp)})
    def selected(self,msg):
        self.goal_sequence+=1;self.goal=(msg.pose.position.x,msg.pose.position.y)
        self.goals.append((self.goal_sequence,*self.goal))
        self.write('selected_goal',message_to_ordereddict(msg))
    def graph(self):
        for topic in ['/cmd_vel','/cmd_vel_nav','/scanplanner/cmd_vel']:
            self.write('publishers',{'topic':topic,'publishers':[{'node':p.node_name,'gid':list(p.endpoint_gid)} for p in self.get_publishers_info_by_topic(topic)]})
    def sample(self):
        try:tr=self.buffer.lookup_transform('map','base_link',rclpy.time.Time())
        except Exception:return
        p=tr.transform.translation;q=tr.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        gx,gy=self.goal if self.goal else ('','')
        distance=math.hypot(gx-p.x,gy-p.y) if self.goal else ''
        self.csv.writerow([self.get_clock().now().nanoseconds/1e9,self.goal_sequence,p.x,p.y,p.z,yaw,gx,gy,distance])
        self.samples.append((p.x,p.y,self.goal_sequence))
        ps=PoseStamped();ps.header.frame_id='map';ps.header.stamp=tr.header.stamp
        ps.pose.position.x=p.x;ps.pose.position.y=p.y;ps.pose.position.z=p.z;ps.pose.orientation=q
        self.path.poses.append(ps)
        if len(self.path.poses)>18000:self.path.poses=self.path.poses[::2]
        self.path.header.stamp=ps.header.stamp;self.pub.publish(self.path)
    def finish(self):
        self.events.close();self.csvfile.close()
        if not self.samples:return
        pts=[(x,y) for x,y,_ in self.samples]+[(x,y) for _,x,y in self.goals]
        xmin=min(x for x,y in pts)-.5;xmax=max(x for x,y in pts)+.5
        ymin=min(y for x,y in pts)-.5;ymax=max(y for x,y in pts)+.5
        scale=min(900/(xmax-xmin),700/(ymax-ymin))
        def xy(x,y):return (40+(x-xmin)*scale,40+(ymax-y)*scale)
        out=['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="800"><rect width="100%" height="100%" fill="white"/>']
        for a,b in zip(self.samples,self.samples[1:]):
            x1,y1=xy(*a[:2]);x2,y2=xy(*b[:2]);color=f'hsl({(b[2]*67)%360},75%,40%)'
            out.append(f'<path d="M{x1},{y1} L{x2},{y2}" fill="none" stroke="{color}" stroke-width="2"/>')
        for seq,x,y in self.goals:
            px,py=xy(x,y);out.append(f'<circle cx="{px}" cy="{py}" r="5" fill="none" stroke="black"/><text x="{px+7}" y="{py}">G{seq}</text>')
        out.append('<text x="20" y="785">Actual trajectory by goal sequence; circles = dispatched frontier goals; map coordinates, no obstacle overlay</text></svg>')
        (self.directory/'overview.svg').write_text(''.join(out))
        print(f'Saved trajectory.csv, events.jsonl, overview.svg in {self.directory}',flush=True)
def main():
    rclpy.init();node=Recorder()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:node.finish();node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
