#!/usr/bin/env python
import ssl
ssl._create_default_https_context = ssl._create_unverified_context

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tensorflow as tf
#from keras.applications.resnet50 import ResNet50, preprocess_input, decode_predictions
#from keras.applications.vgg16 import VGG16,preprocess_input, decode_predictions
#from keras.applications.vgg19 import VGG19,preprocess_input, decode_predictions
from keras.applications.xception import Xception,preprocess_input, decode_predictions


#model = ResNet50(weights='/usr/local/lib/python2.7/dist-packages/keras/models/resnet50_weights_tf_dim_ordering_tf_kernels.h5')
#model = VGG16(weights='/usr/local/lib/python2.7/dist-packages/keras/models/vgg16_weights.h5')
#model = VGG19(weights='/usr/local/lib/python2.7/dist-packages/keras/models/vgg19_weights.h5')
model = Xception(weights='/usr/local/lib/python2.7/dist-packages/keras/models/xception_weights_tf_dim_ordering_tf_kernels.h5')


model._make_predict_function()
graph = tf.get_default_graph()
target_size = (224, 224)

rospy.init_node('classify', anonymous=True)
#These should be combined into a single message
pub = rospy.Publisher('object_detected', String, queue_size = 1)
bridge = CvBridge()

msg_string = String()


def callback(image_msg):

    #First convert the image to OpenCV image 
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    cv_image = cv2.resize(cv_image, target_size)  # resize image 
    
    #Segment image
    ##################################################
    #"""
    original = cv_image.copy()
    gray = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    canny = cv2.Canny(blurred, 120, 255, 1)
    kernel = np.ones((15,15),np.uint8)
    dilate = cv2.dilate(canny, kernel, iterations=1)

    # Find contours
    cnts = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    #catch multiple objects in image
    objects=[]
    for c in cnts:
        x,y,w,h = cv2.boundingRect(c)
        cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2)
        #here ROI stores last object only
        ROI = original[y:y+h, x:x+w]
        objects.append(ROI)
        
        
    cv2.imshow('Original', cv_image)
    cv2.imshow('Contours', original) 
    cv2.imshow('extracted', objects[0]) 
    #"""
    ############################################################
    #np_image = np.asarray(cv_image)
    np_image = np.asarray(objects[0])               # read as np array
    np_image = np.expand_dims(np_image, axis=0)   # Add another dimension for tensorflow
    np_image = np_image.astype(float)             # preprocess needs float64 and img is uint8
    np_image = preprocess_input(np_image)         # Regularize the data

    global graph                                  # This is a workaround for asynchronous execution
    with graph.as_default():
       preds = model.predict(np_image)            # Classify the image
       
       pred_string = decode_predictions(preds, top=20)[0]   # Decode top 1 predictions  

       
       if pred_string[0][2] >0.1:
           msg_string.data = pred_string[0][1]
           pub.publish(msg_string)
           print(pred_string[0][1])
           print(pred_string[0][2])
           print('--------------------')
       else:
           print('No Object detected')
        
    cv2.waitKey(2000)
#simulation
#rospy.Subscriber("/camera/rgb/image_raw", Image, callback, queue_size = 1, buff_size = 16777216)
rospy.Subscriber("/camera/rgb/image_color", Image, callback, queue_size = 1, buff_size = 16777216)




while not rospy.is_shutdown():
  rospy.spin()