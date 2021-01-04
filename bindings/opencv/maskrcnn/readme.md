# MaskR-CNN Example

### Overview
This example  demonstrates object detection on a combination between the depth frame and the IR frame using the MaskR-CNN object detection example from OpenCV and the adiTOF SDK.

OpenCV 3.4.3 is mandatory for the program to compile and run.
This example needs access to 3 configuration files: COCO object class LABELS and two TENSORFLOW model configuration files, based on COCO datase. 

We tested the example using the following cofiguration files: **mscoco_labels.names**, **frozen_inference_graph.pb and mask_rcnn_inception_v2_coco_2018_01_28.pbtxt**. 

To get TENSORFLOW  model configuration files choose between the following two methods:
* use the  following script:
(**https://github.com/opencv/opencv_extra/blob/master/testdata/dnn/download_models.py**). 
* download one of the archives from the link below and get the *.pb* file:
(**https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1_detection_zoo.md**) 
and download the *.pbtxt* file from the following link: (**https://github.com/opencv/opencv_extra/blob/master/testdata/dnn/mask_rcnn_inception_v2_coco_2018_01_28.pbtxt**)

Afterward copy the two TENSORFLOW model configuration files (**frozen_inference_graph.pb and mask_rcnn_inception_v2_coco_2018_01_28.pbtxt**) in the same folder with the Executable (**aditof-opencv-maskrcnn.exe**). 

For running the program with different configuration files use the parameters: *model, config and classes*. 
For example:
```console
 --model=local_path\aditof_sdk\bindings\python\examples\maskr_cnn\frozen_inference_graph.pb --config=local_path\aditof_sdk\bindings\python\examples\maskr_cnn\mask_rcnn_inception_v2_coco_2018_01_28.pbtxt --classes=local_path\aditof_sdk\bindings\python\examples\maskr_cnn\object_detection_classes_coco.txt
```

![Display Image](/doc/img/maskrcnn_cpp.png) 
