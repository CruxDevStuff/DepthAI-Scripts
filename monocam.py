import depthai as dai
import depthai_sdk as dai_sdk
import cv2

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
rgb_cam = pipeline.create(dai.node.ColorCamera)
videoEncoder = pipeline.create(dai.node.VideoEncoder)

controlIn = pipeline.create(dai.node.XLinkIn)
videoMjpegOut = pipeline.create(dai.node.XLinkOut)

controlIn.setStreamName('control')
videoMjpegOut.setStreamName('video')

# Properties
rgb_cam.setVideoSize(1280,960)
rgb_cam.setFps(35)
videoEncoder.setDefaultProfilePreset(rgb_cam.getFps(), dai.VideoEncoderProperties.Profile.MJPEG)

# Linking
rgb_cam.video.link(videoEncoder.input)
controlIn.out.link(rgb_cam.inputControl)
videoEncoder.bitstream.link(videoMjpegOut.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Get data queues
    controlQueue = device.getInputQueue('control')
    videoQueue = device.getOutputQueue('video')

    #ctrl = dai.CameraControl()
    #ctrl.setManualFocus(0)
    #controlQueue.send(ctrl)

    while True:
        
        videoFrames = videoQueue.tryGetAll()
        for videoFrame in videoFrames:
            # Decode JPEG
            frame = cv2.imdecode(videoFrame.getData(), cv2.IMREAD_UNCHANGED)
            # Display
            print(rgb_cam.getFps())
            cv2.imshow('RGB Mono', frame)

        # Update screen (1ms pooling rate)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        
