import rospy
import cv2
import depthai as dai
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

pipeline = dai.Pipeline()
bridge = CvBridge()
manip = pipeline.createImageManip()

def initStereoCam():
     # Set up left and right cameras
    monoLeft = getMonoCamera(pipeline, isLeft = True)
    monoRight = getMonoCamera(pipeline, isLeft = False)

    # Set output Xlink for left camera
    xoutLeft = pipeline.createXLinkOut()
    xoutLeft.setStreamName("left")

    # Set output Xlink for right camera
    xoutRight = pipeline.createXLinkOut()
    xoutRight.setStreamName("right")
 
    # Attach cameras to output Xlink
    monoLeft.out.link(xoutLeft.input)
    monoRight.out.link(xoutRight.input)


def initRGBCam():
    # Define sources and outputs
    rgb_cam = pipeline.create(dai.node.ColorCamera)
    videoEncoder = pipeline.create(dai.node.VideoEncoder)

    controlIn = pipeline.create(dai.node.XLinkIn)
    videoMjpegOut = pipeline.create(dai.node.XLinkOut)

    controlIn.setStreamName('control')
    videoMjpegOut.setStreamName('video')

    # Properties
    rgb_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    rgb_cam.setInterleaved(False)
    rgb_cam.setFps(40)
    videoEncoder.setDefaultProfilePreset(rgb_cam.getFps(), dai.VideoEncoderProperties.Profile.MJPEG)

    # Linking
    rgb_cam.video.link(videoEncoder.input)
    controlIn.out.link(rgb_cam.inputControl)
    videoEncoder.bitstream.link(videoMjpegOut.input)



def getFrame(queue):
    frame = queue.get()
    return frame.getCvFrame()

def getMonoCamera(pipeline, isLeft):
    mono = pipeline.createMonoCamera()
    mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
    
    if isLeft:
        mono.setBoardSocket(dai.CameraBoardSocket.LEFT)
    else :
        mono.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    return mono


def publisher():
    
    initStereoCam()
    initRGBCam()

    left_pub = rospy.Publisher('/stereo/left/image_raw', Image, queue_size=1)
    right_pub = rospy.Publisher('/stereo/right/image_raw', Image, queue_size=1)
    rgb_pub = rospy.Publisher('/stereo/rgb/image_raw', Image, queue_size=1)
    rospy.init_node('stereo', anonymous=False)
    rate = rospy.Rate(1000)


    while not rospy.is_shutdown():
        
        with dai.Device(pipeline) as device:

            controlQueue = device.getInputQueue('control')
            rgbQueue = device.getOutputQueue('video')
            leftQueue = device.getOutputQueue(name="left", maxSize=1)
            rightQueue = device.getOutputQueue(name="right", maxSize=1)
            
            ctrl = dai.CameraControl()
            ctrl.setManualFocus(0)
            controlQueue.send(ctrl)
            
            while True:
                leftFrame = getFrame(leftQueue)
                rightFrame = getFrame(rightQueue)
                rgb_frames = getFrame(rgbQueue)

                rgb_frame = cv2.imdecode(rgb_frames, cv2.IMREAD_UNCHANGED)

                imOut = np.hstack((leftFrame, rightFrame))

                left_img = bridge.cv2_to_imgmsg(leftFrame, 'mono8')
                right_img = bridge.cv2_to_imgmsg(rightFrame, 'mono8')
                rgb_img = bridge.cv2_to_imgmsg(rgb_frame, 'bgr8')
                
                left_pub.publish(left_img)
                right_pub.publish(right_img)
                rgb_pub.publish(rgb_img)
                
                key = cv2.waitKey(1)
                if key == ord('q'):
                    break


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
    
    
            

