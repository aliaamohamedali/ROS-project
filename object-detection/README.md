
# Object detecion and recognition (Old Version)

## Steps running on Kinect camera

0. Place the object_recognition folder in ros workspace src folder
1. Place your model weights **.h5** file
2. Run the roscore topic
   - **roscore**
3. Run the required driver (ex : Freenect driver)
   - **roslaunch freenect_launch freenect.launch**
4. Run the rgb image view topic the script subscibes on to track what the camera is detecting on real time
   - **rosrun image_view image_view image:=/camera/rgb/image_raw**
5. Run the script
   - **rosrun object_recognition classify.py**
6. Run the topic the scipt publishes its recongintion on as a string
   - **rostopic echo /object_detected**
  
