# 3D Time of Flight : Python Challenge

### Overview

The gesture recognition example offers a GUI to load and process depth images
acquired from a 3D ToF camera. 
The application takes an image of a hand as input and applies a processing scheme
to it, in order to determine the number of extended fingers from the input image.
For this challenge, participants will have to improve the finger detection algorithm in 
order to obtain valid results on all given images.

### Running the application

Open the gesture_recognition directory in a terminal window or in a Python IDE of your choice.
First, make sure all the required python packages are installed. To do this, run:
```
pip install -r requirements.txt
```
To open the application, run the python script process.py.
Click the "Load" button to open an image and the "Process" button to apply the
finger detection algorithm to it. 

### Current state of software

The python file "process.py" handles the graphical elements displayed in the interface,
as well the processing task. 
When processing an image, the algorithm follows these steps:
 * convert input image to binary image, using the input's histogram
 * detect the hand and its center of mass
 * find the contour of the hand with extremities using the ConvexHull method
 * compute the fingertips distances to the center of the hand and detect the gesture

The 6 images are given as test cases for this challenge. Each file is named after the
number of extended fingers that are to be detected. The algorithm may already present accurate
results on some of these test cases. The goal is to have it perform adequately for all the cases.

### The Challenge

How many fingers am I holding up ?

The functions that handle the algorithm are the following:
 * _detect_hand() - line 89
 * _count_fingers() - line 103

Participants will have to edit these in order to modify the algorithm.

### Submissions

To submit your results, please archive the gesture_recognition directory and send 
us the .zip file, with the subject: "3D ToF Python Challenge"

Address : cristina.suteu@analog.com


