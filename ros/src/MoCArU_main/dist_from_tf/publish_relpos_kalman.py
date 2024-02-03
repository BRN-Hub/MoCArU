#!/usr/bin/env python3

import sys
import tf
import tf.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import rospy
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math
import numpy as np

robotnum = sys.argv[1]
print(f"The Robot {robotnum} detector has started. ")

class Camera:
    def __init__(self, camera_number):
        self.camera_number = camera_number
        self.xp, self.yp, self.zp = 0, 0, 0  # camera's coordinate
        self.xr, self.yr, self.zr = 0, 0, 0  # robot's coordinate
        self.qxp, self.qyp, self.qzp, self.qwp = 0, 0, 0, 0  # camera's quarternion
        self.qxr, self.qyr, self.qzr, self.qwr = 0, 0, 0, 0  # robot's quarternion
        self.globmark_seen = False  # whether the Global marker has been seen
        self.globmark_lasttime, self.robmark_lasttime = rospy.Time(0), rospy.Time(0)  # last time that markers are seen

        self.dx, self.dy, self.dz, self.dwz = 0, 0, 0, 0  #computed pose

    def update_cam_info(self,basis):
        self.xp = basis.translation.x
        self.yp = basis.translation.y
        self.zp = basis.translation.z
        self.qxp = basis.rotation.x
        self.qyp = basis.rotation.y
        self.qzp = basis.rotation.z
        self.qwp = basis.rotation.w

    def update_rob_info(self,basis):
        self.xr = basis.translation.x
        self.yr = basis.translation.y
        self.zr = basis.translation.z
        self.qxr = basis.rotation.x
        self.qyr = basis.rotation.y
        self.qzr = basis.rotation.z
        self.qwr = basis.rotation.w

    def compute_pose(self):
        xcp_0, ycp_0, zcp_0, rotzp_0 = findCameraFrame(self.xp, self.yp, self.zp, self.qxp, self.qyp, self.qzp, self.qwp)
        xcr_0, ycr_0, zcr_0, rotzr_0 = findCameraFrame(self.xr, self.yr, self.zr, self.qxr, self.qyr, self.qzr, self.qwr)  
        self.dx, self.dy, self.dz, self.dwz = computeDiff(xcp_0, ycp_0, zcp_0, rotzp_0, xcr_0, ycr_0, zcr_0, rotzr_0)

#matrices for Kalman filtering
P_k = np.array([[1.0,   0,   0,   0],
                [  0, 1.0,   0,   0],
                [  0,   0, 1.0,   0],
                [  0,   0,   0, 1.0]])
S_k = np.array([[1.0,   0,   0,   0],
                [  0, 1.0,   0,   0],
                [  0,   0, 1.0,   0],
                [  0,   0,   0, 1.0]])*0.1
state_x = np.array([[0],
                    [0],
                    [0],
                    [0]])

prev_odom_x = 0 #used only for odom
prev_odom_y = 0 #used only for odom
prev_odom_z = 0 #used only for odom
prev_odom_wz = 0 #used only for odom

def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x*180/3.1415, pitch_y*180/3.1415, yaw_z*180/3.1415

def findCameraFrame(x, y, z, qx, qy, qz, qw):
    rotx, roty, rotz = euler_from_quaternion(qx, qy, qz, qw)
    xc = -x*np.cos(rotz*3.1415/180)-y*np.sin(rotz*3.1415/180)
    yc = +x*np.sin(rotz*3.1415/180)-y*np.cos(rotz*3.1415/180)
    zc = -z
    return xc, yc, zc, rotz

def mapto180(x):
    if x <= 180:
        return x
    elif x >= 180:
        return x-360

def mapto360(x):
    if x >= 0 and x <= 360:
        return x
    elif x > 360:
        return x - 360
    else:
        return x + 360       

def computeDiff(xcp, ycp, zcp, rotzp, xcr, ycr, zcr, rotzr):
    dx_cambasis, dy_cambasis, dz, dwz = xcr-xcp, ycr-ycp, zcr-zcp, mapto180((rotzr-rotzp)%360) #now dx and dy is based on the basis on the global pattern but parallel to camera
    dx = dx_cambasis*np.cos(rotzp*3.1415/180)-dy_cambasis*np.sin(rotzp*3.1415/180)
    dy = dx_cambasis*np.sin(rotzp*3.1415/180)+dy_cambasis*np.cos(rotzp*3.1415/180) #so it is transformed back
    return dx, dy, dz, dwz

def get_distance(x, y, z):
    return (x**2+y**2+z**2)**0.5

def combineByKalman(x, y, z, wz, dist):
    global P_k, S_k, state_x
    state_prev = state_x.copy()
    R_k = np.array([[0.1228*dist,   0,   0,   0],
                    [  0, 0.1204*dist,   0,   0],
                    [  0,   0, 0.1091*dist,   0],
                    [  0,   0,   0, 1.1584*dist]])
    S_k = np.add(P_k,R_k)
    K_k = P_k@np.linalg.inv(S_k)
    tempF = np.subtract(np.array([[x],[y],[z],[wz]]),state_x)
    tempF[3] = mapto180((tempF[3])%360)
    state_x = state_x+K_k@tempF
    P_k = np.subtract(np.eye(4),K_k)@P_k
    # print(state_prev[3], state_x[3])
    return state_x[0][0], state_x[1][0], state_x[2][0], state_x[3][0]

def callback_odom(data):
    global P_k, state_x, prev_odom_x, prev_odom_y, prev_odom_z, prev_odom_wz
    new_odom_x = data.pose.pose.position.x
    new_odom_y = data.pose.pose.position.y
    new_odom_z = data.pose.pose.position.z
    _, _, new_odom_wz = euler_from_quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
    # check whether any of the velocities is not zero
    if (   abs(data.twist.twist.linear.x)>=0.005 
        or data.twist.twist.linear.y!=0 
        or data.twist.twist.linear.z!=0 
        or data.twist.twist.angular.x!=0 
        or data.twist.twist.angular.y!=0 
        or abs(data.twist.twist.angular.z)>=0.05
    ):
        fwd_or_bck = 1 if data.twist.twist.linear.x>0 else -1
        r = fwd_or_bck*((new_odom_x-prev_odom_x)**2+(new_odom_y-prev_odom_y)**2)**0.5
        azim = state_x[3][0]*3.1415/180
        state_x = np.add(state_x,np.array([[r*np.sin(azim)],
                                        [r*np.cos(azim)],
                                        [0],
                                        [-math.atan2(math.sin(new_odom_wz-prev_odom_wz), math.cos(new_odom_wz-prev_odom_wz))]]))
        P_k = np.add(P_k,np.array([[0.1228,    0,   0,   0],
                                    [  0, 0.1204,   0,   0],
                                    [  0,   0, 0.1091,   0],
                                    [  0,   0,   0, 1.1584]]))
    prev_odom_x = new_odom_x
    prev_odom_y = new_odom_y
    prev_odom_z = new_odom_z
    prev_odom_wz = new_odom_wz
    DynamicTFBroadcaster(state_x[0][0], state_x[1][0], state_x[2][0], state_x[3][0], "turtle_kalman_t"+robotnum)

def callback(data, allcamera_info):

    global state_x

    for camera_info in allcamera_info:
        if data.transforms[0].child_frame_id == f"SSP{camera_info.camera_number}_camera":
            camera_info.globmark_lasttime = data.transforms[0].header.stamp
            camera_info.globmark_seen = True
            basis = data.transforms[0].transform
            camera_info.update_cam_info(basis)
            break
        if camera_info.globmark_seen and data.transforms[0].child_frame_id == f"SSP{camera_info.camera_number}_robot{robotnum}":
            camera_info.robmark_lasttime = data.transforms[0].header.stamp
            camera_info.robmark_seen = True
            basis = data.transforms[0].transform
            camera_info.update_rob_info(basis)
            camera_info.compute_pose()
            x, y, z, dw = combineByKalman(camera_info.dx, camera_info.dy, camera_info.dz, camera_info.dwz, get_distance(camera_info.xr, camera_info.yr, camera_info.zr))
            DynamicTFBroadcaster(x, y, z, dw, "turtle_kalman_t"+robotnum)
            break

    # adding reliable coordinates to the list. then removing outliers. averaging the rest
    tolerance = 1 # how long (s) before the camera is deemed frozen or not detecting both frames
    num=0

def callback_scan(data):
    # print(rospy.get_rostime())
    data.header.stamp = rospy.Time.now()
    data.header.frame_id = 'turtle'+robotnum+'_scan'
    pub = rospy.Publisher('/scan_tb'+robotnum, LaserScan, queue_size = 10)

    pub.publish(data)

def DynamicTFBroadcaster(dx, dy, dz, dwz,frame_id):
    pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=3)
    
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "base"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = frame_id
    t.transform.translation.x = dx
    t.transform.translation.y = dy
    t.transform.translation.z = dz

    q = quaternion_from_euler(0, 0, -dwz*3.1415/180)
    
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    tfm = tf.msg.tfMessage([t])
    pub_tf.publish(tfm)
    br = tf.TransformBroadcaster()

    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "turtle"+robotnum+"_scan",
                     frame_id)

    # In case that the orientation of the LIDAR of each robot is not the same, then the if statements below are required. 
    # if robotnum=="1":
    #     br.sendTransform((0, 0, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, -np.pi),
    #                  rospy.Time.now(),
    #                  "turtle"+robotnum+"_scan",
    #                  frame_id)

    # if robotnum=="2":
    #     br.sendTransform((0, 0, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, +np.pi/2),
    #                  rospy.Time.now(),
    #                  "turtle"+robotnum+"_scan",
    #                  frame_id)

def DynamicMarkerBroadcaster(dx, dy, dz, sx, sy, sz, topic_name):
    marker_pub = rospy.Publisher(topic_name, Marker, queue_size = 2)

    marker = Marker()

    marker.header.frame_id = "base"
    marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 2
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = sx
    marker.scale.y = sy
    marker.scale.z = sz

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.5

    # Set the pose of the marker
    marker.pose.position.x = dx
    marker.pose.position.y = dy
    marker.pose.position.z = dz
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker_pub.publish(marker)
    
def listener_and_publisher(allcamera_info):
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("tf", TFMessage, callback, allcamera_info)
    rospy.Subscriber('TurtleBot'+robotnum+'/odom', Odometry, callback_odom)
    rospy.Subscriber('TurtleBot'+robotnum+'/scan', LaserScan, callback_scan) #only to publish the same info with new time
    num_cameras = int(rospy.get_param('~camera'))
    print(num_cameras)
    rospy.spin()

if __name__ == '__main__':
    CAMERA_NUMBER = 10
    allcamera_info = [Camera(i) for i in range(CAMERA_NUMBER)]
    listener_and_publisher(allcamera_info)
