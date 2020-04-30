# MaskR-CNN Example

### Overview
This example  demonstrates object detection on a combination between the depth frame and the IR frame using the MaskR-CNN object detection example from OpenCV and the adiTOF SDK.

This example needs access to 3 configuration files: COCO object class LABELS and two TENSORFLOW model configuration files, based on COCO datase. 

We tested the example using the following cofiguration files: **object_detection_classes_coco.txt**, **frozen_inference_graph.pb and mask_rcnn_inception_v2_coco_2018_01_28.pbtxt**. 

To get TENSORFLOW  model configuration files (**frozen_inference_graph.pb and mask_rcnn_inception_v2_coco_2018_01_28.pbtxt**) choose between the following two methods:
* use the following script:
(**https://github.com/opencv/opencv_extra/blob/master/testdata/dnn/download_models.py**). 
* download one of the archives from the link below and get the *.pb* and *.pbtxt* files from that archive.
(**https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md**)

For running the python program use:
```console
python maskr_cnn.py --model local_path\aditof_sdk\bindings\python\examples\maskr_cnn\frozen_inference_graph.pb --config local_path\aditof_sdk\bindings\python\examples\maskr_cnn\mask_rcnn_inception_v2_coco_2018_01_28.pbtxt --classes local_path\aditof_sdk\bindings\python\examples\maskr_cnn\object_detection_classes_coco.txt
```

![Display Image](/doc/img/maskrcnn_python.png)
