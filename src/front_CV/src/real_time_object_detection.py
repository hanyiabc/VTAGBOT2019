#!/usr/bin/env python
# USAGE
# python real_time_object_detection.py --prototxt MobileNetSSD_deploy.prototxt.txt --model MobileNetSSD_deploy.caffemodel

# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2
import roslib
import sys
import rospy
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospkg
import os
#maxPixel = rospy.get_param("/maxPixel")
global frame
global image_pub
global bridge
global net
maxPixel = 800
vs = VideoStream(src=0).start()
objectDetected = False
bridge = CvBridge()


def front_image_callback(data):
	global frame
	global image_pub
	global bridge
	global net
	init_confidence = 0.2
	# initialize the list of class labels MobileNet SSD was trained to
	# detect, then generate a set of bounding box colors for each class
	CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
		"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
		"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
		"sofa", "train", "tvmonitor", "watermelon"]
	COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
	frame = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
	frame = imutils.resize(frame, width=maxPixel)

	# grab the frame dimensions and convert it to a blob
	(h, w) = frame.shape[:2]
	blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
		0.007843, (300, 300), 127.5)

	# pass the blob through the network and obtain the detections and
	# predictions
	net.setInput(blob)
	detections = net.forward()
	objectDetected = False
	# loop over the detections
	for i in np.arange(0, detections.shape[2]):
		# extract the confidence (i.e., probability) associated with
		# the prediction
		confidence = detections[0, 0, i, 2]

		# filter out weak detections by ensuring the `confidence` is
		# greater than the minimum confidence
		if confidence > init_confidence:
			# extract the index of the class label from the
			# `detections`, then compute the (x, y)-coordinates of
			# the bounding box for the object
			idx = int(detections[0, 0, i, 1])
			box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
			(startX, startY, endX, endY) = box.astype("int")


			# draw the prediction on the frame
			label = "{}: {:.2f}%".format(CLASSES[idx],
				confidence * 100)
			cv2.rectangle(frame, (startX, startY), (endX, endY),
				COLORS[idx], 2)
			y = startY - 15 if startY - 15 > 15 else startY + 15
			cv2.putText(frame, label, (startX, y),
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
			size = (endX - startX)*(endY-startY)
	image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))		# update the FPS counter

def CVControl():
	global image_pub
	global frame
	global net
	rspkg = rospkg.RosPack()
 	prototxt_path = os.path.join(rspkg.get_path('front_CV'),"src","MNSSD_melon_1200000.prototxt")
 	caffemodel_path = os.path.join(rspkg.get_path('front_CV'),"src","MNSSD_melon_1200000.caffemodel")
	# prototxt_path = "~/agtrec/agbot_deploy/src/melonCV/src/NSSD_melon_1200000.prototxt"
	# caffemodel_path = "~/agtrec/agbot_deploy/src/melonCV/src/MNSSD_melon_1200000.caffemodel"


	# load our serialized model from disk
	print("[INFO] loading model...")
	net = cv2.dnn.readNetFromCaffe(prototxt_path, caffemodel_path)

	# initialize the video stream, allow the cammera sensor to warmup,
	# and initialize the FPS counter
	print("[INFO] starting video stream...")

	time.sleep(2.0)
	fps = FPS().start()




	rospy.init_node("real_time_object_detection", anonymous = True)
	image_pub = rospy.Publisher("front_cv",Image, queue_size = 10)
	image_sub = rospy.Subscriber("front_camera/front_image_raw", Image, front_image_callback)
	rate = rospy.Rate(30)

	frame_count = 0
	prev_Y = 0
	center = [0,0]
	# loop over the frames from the video stream
	rospy.spin()


if __name__ == '__main__':
	CVControl()
# stop the timer and display FPS information
# fps.stop()
# print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
# print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
