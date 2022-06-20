import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import apriltag

cap = cv2.VideoCapture(0)
bridge = CvBridge()


def publisher():
    pub = rospy.Publisher('/usb_cam', Image, queue_size=1)
    rospy.init_node('image', anonymous=False)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        print("[INFO] detecting AprilTags...")
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(gray)
        print("[INFO] {} total AprilTags detected".format(len(results)))

        for r in results:
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            cv2.line(gray, ptA, ptB, (0, 255, 0), 2)
            cv2.line(gray, ptB, ptC, (0, 255, 0), 2)
            cv2.line(gray, ptC, ptD, (0, 255, 0), 2)
            cv2.line(gray, ptD, ptA, (0, 255, 0), 2)

            (cX, cY) = (int(r.center[0]),
                            int(r.center[1]))

            cv2.circle(gray, (cX, cY), 5, (0, 0, 255), -1)
            print(cX)
            print(cY)

        if not ret:
            break

        msg = bridge.cv2_to_imgmsg(gray, 'mono8')
        pub.publish(msg)

        if cv2.waitKey(1) == ord("q"):
            break

        if rospy.is_shutdown():
            cap.release()


if __name__ == '__main__':
    try:
        publisher()

    except rospy.ROSInterruptException:
        pass
