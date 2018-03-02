# Axis camera image driver. Based on:
# https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/axis_camera
# /axis.py
#

import base64

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import threading
import urllib.request

###import rospy 
from sensor_msgs.msg import CompressedImage, CameraInfo
###import camera_info_manager

class StreamThread(threading.Thread):
    def __init__(self, axis):
        threading.Thread.__init__(self)
        self.axis = axis
        self.daemon = True
        self.timeoutSeconds = 2.5

    def run(self):
        while(True):
            self.stream()

    def stream(self):
        while(True):
            self.formURL()
            if self.authenticateAndOpen():
                self.publishFramesContinuously()
###            rospy.sleep(2) # if stream stays intact we shouldn't get to this

    def formURL(self):
        self.url = 'http://%s:%s/mjpg/video.mjpg' % (self.axis.hostname, self.axis.hostport)
        self.url += "?fps=0&resolution=%dx%d" % (self.axis.width, self.axis.height)
###        rospy.loginfo('opening ' + str(self.axis))

    def authenticate(self):
        '''only try to authenticate if user/pass configured.  I have not
        used this method (yet).'''
        if self.axis.password != '' and self.axis.username != '':
            # create a password manager
            password_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()

            # Add the username and password, use default realm.
            #top_level_url = "http://" + self.axis.hostname + self.axis.hostport
            top_level_url = self.url
            password_mgr.add_password(None, top_level_url, self.axis.username, 
                                                            self.axis.password)
            if self.axis.use_encrypted_password :
                handler = urllib.request.HTTPDigestAuthHandler(password_mgr)
            else:
                handler = urllib.request.HTTPBasicAuthHandler(password_mgr)

            # create "opener" (OpenerDirector instance)
            opener = urllib.request.build_opener(handler)

            # ...and install it globally so it can be used with urlopen.
            urllib.request.install_opener(opener)
    
    def openURL(self):
        '''Open connection to Axis camera using http'''
        try:
            self.fp = urllib.request.urlopen(self.url, timeout=self.timeoutSeconds)
            return(True)
        except urllib.request.URLError as e:
###            rospy.logwarn('Error opening URL %s Reason: %s. Looping until camera appears' % (self.url, e.reason))
            # print e.reason
            return(False)

    def authenticateAndOpen(self):
        '''only try to authenticate if user/pass configured.  I have not
        used this method (yet).'''
        if self.axis.password != '' and self.axis.username != '':
            print("Creating req")
            req = urllib.request.Request(self.url)
            credentials = ("%s:%s" % (self.axis.username, self.axis.password))
            encoded = base64.b64encode(credentials.encode("ascii"))
            req.add_header("Authorization","Basic %s" % encoded.decode("ascii"))
            
        try:
            self.fp = urllib.request.urlopen(req)
            print("opening URL")
            return(True)
        except urllib.request.URLError as e:
            print("error opening URL")
###            rospy.logwarn('Error opening URL %s Reason: %s. Looping until camera appears' % (self.url, e.reason))
            # print e.reason
            return(False)

    def publishFramesContinuously(self):
        '''Continuous loop to publish images'''
        while(True):
            try:
                self.findBoundary()
                self.getImage()
                self.publishMsg()
###                self.publishCameraInfoMsg()
            except Exception as e:
                print("Exception:", e)
###                rospy.loginfo('Timed out while trying to get message.')
                break

    def findBoundary(self):
        '''The string "--myboundary" is used to denote the start of an image in
        Axis cameras'''
        print ("finding boundary")
        while(True):
            boundary = self.fp.readline().decode("ascii")
            if boundary=='--myboundary\r\n':
                print ("found a boundary")
                break

    def getImage(self):
        '''Get the image header and image itself'''
        self.getHeader()
        self.getImageData()

    def getHeader(self):
        self.header = {}
        print("getting header")
        while(True):
            line = self.fp.readline().decode("ascii")
            if line == "\r\n":
                break
            line = line.strip()
            parts = line.split(": ", 1)
            try:
                self.header[parts[0]] = parts[1]
            except:
                print("Problem getting header")
###                rospy.logwarn('Problem encountered with image header.  Setting '
###                                                    'content_length to zero')
                self.header['Content-Length'] = 0 # set content_length to zero if 
                                            # there is a problem reading header
        print(self.header)
        self.content_length = int(self.header['Content-Length'])

    def getImageData(self):
        '''Get the binary image data itself (ie. without header)'''
        if self.content_length>0:
            self.img = self.fp.read(self.content_length)
            self.fp.readline() # Read terminating \r\n and do nothing with it

    def publishMsg(self):
        '''Publish jpeg image as a ROS message'''
###        self.msg = CompressedImage()
        self.msg = Image()
###        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.axis.frame_id
###        self.msg.format = "jpeg"
        self.msg.width = self.axis.width
        self.msg.height = self.axis.height
        self.msg.encoding = "bgr8"
        self.msg.data = self.img
        self.msg.step = int(len(self.msg.data)/self.msg.height)
        self.axis.publisher_.publish(self.msg)

    def publishCameraInfoMsg(self):
        '''Publish camera info manager message'''
###        cimsg = self.axis.cinfo.getCameraInfo()
        cimsg.header.stamp = self.msg.header.stamp
        cimsg.header.frame_id = self.axis.frame_id
        cimsg.width = self.axis.width
        cimsg.height = self.axis.height
###        self.axis.caminfo_pub.publish(cimsg)

class Axis(Node):
    def __init__(self, hostname, hostport, username, password, width, height,
                 frame_id, camera_info_url, use_encrypted_password):
        self.hostname = hostname
        self.hostport = hostport
        self.username = username
        self.password = password
        self.width = width
        self.height = height
        self.frame_id = frame_id
        self.camera_info_url = camera_info_url
        self.use_encrypted_password = use_encrypted_password
        
        super().__init__('axis_camera')

    
        # generate a valid camera name based on the hostname
###        self.cname = camera_info_manager.genCameraName(self.hostname)
###        self.cinfo = camera_info_manager.CameraInfoManager(cname = self.cname,
###                                                   url = self.camera_info_url)
###        self.cinfo.loadCameraInfo()         # required before getCameraInfo()
        self.st = None
        self.st = StreamThread(self)
        self.st.start()
###        self.pub = rospy.Publisher("image_raw/compressed", CompressedImage, self, queue_size=1)
###        self.caminfo_pub = rospy.Publisher("camera_info", CameraInfo, self, queue_size=1)
        self.publisher_ = self.create_publisher(Image, 'image')

    def __str__(self):
        """Return string representation."""
        return(self.hostname + ',' + self.username + ',' + self.password +
                       '(' + str(self.width) + 'x' + str(self.height) + ')')

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        '''Lazy-start the image-publisher.'''
        if self.st is None:
            self.st = StreamThread(self)
            self.st.start()

def main():
###    rospy.init_node("axis_driver")
###    rospy.loginfo('axis_driver HELLO!')

    arg_defaults = {
        'hostname': '192.168.0.64',       # default IP address
        'hostport': '8082',               # default port
        'username': 'root',               # default login name
        'password': 'drrobot',
        'width': 640,
        'height': 480,
        'frame_id': 'axis_camera',
        'camera_info_url': '',
        'use_encrypted_password' : False}
###    args = updateArgs(arg_defaults)
    args = updateArgsMini(arg_defaults)
    rclpy.init(args=args)
    axis_publisher = Axis(**args)
    rclpy.spin(axis_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    axis_publisher.destroy_node()
    rclpy.shutdown()
    
def updateArgsMini(arg_defaults):
    args = {}
    for name, val in arg_defaults.items():
        args[name] = val
    return(args)    

def updateArgs(arg_defaults):
    '''Look up parameters starting in the driver's private parameter space, but
    also searching outer namespaces.  Defining them in a higher namespace allows
    the axis_ptz.py script to share parameters with the driver.'''
    args = {}
    for name, val in arg_defaults.iteritems():
        full_name = rospy.search_param(name)
        if full_name is None:
            args[name] = val
        else:
            args[name] = rospy.get_param(full_name, val)
    # resolve frame_id with tf_prefix (unless already absolute)
    if args['frame_id'][0] != '/':        # not absolute?
        tf_prefix = rospy.search_param('tf_prefix')
        prefix_val = ''
        if tf_prefix is not None:           # prefix defined?
            prefix_val = rospy.get_param(tf_prefix)
            if prefix_val[0] != '/':          # prefix not absolute?
                prefix_val = '/' + prefix_val
        args['frame_id'] = prefix_val + '/' + args['frame_id']
    return(args)

if __name__ == "__main__":
    main()
