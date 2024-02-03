#!/usr/bin/env python3

import sys
import tf
import tf.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import rospy
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
import math
import numpy as np

robotnum = sys.argv[1]  # obtain the robot number specified by the launch file
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

        self.dx, self.dy, self.dz, self.dwz = 0, 0, 0, 0 #computed pose
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
    if x<=180:
        return x
    elif x>=180:
        return x-360    

def computeDiff(xcp, ycp, zcp, rotzp, xcr, ycr, zcr, rotzr):
    dx_cambasis, dy_cambasis, dz, dwz = xcr-xcp, ycr-ycp, zcr-zcp, mapto180((rotzr-rotzp)%360)  #now dx and dy is based on the basis on the global pattern but parallel to camera
    dx = dx_cambasis*np.cos(rotzp*3.1415/180)-dy_cambasis*np.sin(rotzp*3.1415/180)
    dy = dx_cambasis*np.sin(rotzp*3.1415/180)+dy_cambasis*np.cos(rotzp*3.1415/180)
    return dx, dy, dz, dwz

def get_distance(x, y, z):
    return (x**2+y**2+z**2)**0.5

def GaussianMul(mu_list, std_list):
    var_list = [v**2 for v in std_list]
    std_prod = ( 1/sum([1/v for v in var_list]) )**0.5
    mu_prod = sum([m/v for m,v in zip(mu_list,var_list)])*(std_prod**2)
    return mu_prod, std_prod

def combineByGaussian(x_list, y_list, z_list, wz_list, dist_list):
    dx, sx = GaussianMul(x_list, [d*0.1228 for d in dist_list])
    dy, sy = GaussianMul(y_list, [d*0.1204 for d in dist_list])
    dz, sz = GaussianMul(z_list, [d*0.1091 for d in dist_list])
    dwz, swz= GaussianMul(wz_list, [d*1.1584 for d in dist_list])
    return dx, dy, dz, dwz, sx, sy, sz, swz

def combineByAverage(x_list, y_list, z_list, wz_list):
    dx = np.mean(x_list)
    dy = np.mean(y_list)
    dz = np.mean(z_list)
    dwz = np.mean(wz_list)
    return dx, dy, dz, dwz

def callback(data, allcamera_info):
    
    tolerance = 0.5  # how long (s) before the camera is deemed frozen or not detecting both frames
    outlier_threshold = 2  # multiples of standard devition we consider a point not to be an outlier

    for camera_info in allcamera_info:
        if data.transforms[0].child_frame_id == f"SSP{camera_info.camera_number}_camera":
            camera_info.globmark_lasttime = data.transforms[0].header.stamp
            camera_info.globmark_seen = True
            basis = data.transforms[0].transform
            camera_info.update_cam_info(basis)
            break
        if data.transforms[0].child_frame_id == f"SSP{camera_info.camera_number}_robot{robotnum}":
            camera_info.robmark_lasttime = data.transforms[0].header.stamp
            camera_info.robmark_seen = True
            basis = data.transforms[0].transform
            camera_info.update_rob_info(basis)
            break

    for camera_info in allcamera_info:
        camera_info.compute_pose()
    
    # adding reliable coordinates to the list. then removing outliers. averaging the rest
    x_list, y_list, z_list, wz_list = np.array([]), np.array([]), np.array([]), np.array([])
    dist_list = np.array([]) # need this list when using Gaussian combination

    # to store those after getting filtered
    x_list_filtered, y_list_filtered, z_list_filtered, wz_list_filtered = np.array([]), np.array([]), np.array([]), np.array([])
    dist_list_filtered = np.array([]) 
    
    global_lasttime=rospy.Time.now()

    for camera_info in allcamera_info:
        if camera_info.globmark_seen and (global_lasttime - camera_info.robmark_lasttime).to_sec() <= tolerance:
            x_list = np.append(x_list, camera_info.dx)
            y_list = np.append(y_list, camera_info.dy)
            z_list = np.append(z_list, camera_info.dz)
            wz_list = np.append(wz_list, camera_info.dwz)
            dist_list = np.append(dist_list, get_distance(camera_info.xr, camera_info.yr, camera_info.zr))
    
    num = len(x_list) #how many data have been collected so far

    # filtering outliers
    for i in range(num):
        if (    abs(x_list[i]-np.mean(x_list))<=outlier_threshold*np.std(x_list)
            and abs(y_list[i]-np.mean(y_list))<=outlier_threshold*np.std(y_list)
            and abs(z_list[i]-np.mean(z_list))<=outlier_threshold*np.std(z_list)
            and abs(wz_list[i]-np.mean(wz_list))<=outlier_threshold*np.std(wz_list)
        ):
            x_list_filtered = np.append(x_list_filtered, x_list[i])
            y_list_filtered = np.append(y_list_filtered, y_list[i])
            z_list_filtered = np.append(z_list_filtered, z_list[i])
            wz_list_filtered = np.append(wz_list_filtered, wz_list[i])
            dist_list_filtered = np.append(dist_list_filtered, dist_list[i])

    # as long as there are some data
    if len(x_list_filtered)>0:
        
        # if use averaging
        dx, dy, dz, dwz = combineByAverage(x_list_filtered, y_list_filtered, z_list_filtered, wz_list_filtered)
        DynamicTFBroadcaster(dx, dy, dz, dwz, "turtle_avg_t"+robotnum)

        # if use Gaussian product
        dx, dy, dz, dwz, sx, sy, sz, swz = combineByGaussian(x_list, y_list, z_list, wz_list, dist_list)
        DynamicTFBroadcaster(dx, dy, dz, dwz, "turtle_gaussian_t"+robotnum)
        DynamicMarkerBroadcaster(dx, dy, dz, sx, sy, sz, "/turtle_marker_gaussian")

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
    rospy.spin()

if __name__ == '__main__':
    CAMERA_NUMBER = 10
    allcamera_info = [Camera(i) for i in range(CAMERA_NUMBER)]
    listener_and_publisher(allcamera_info)

