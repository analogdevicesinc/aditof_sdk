# MaskR-CNN Example

### Overview
This example  demonstrates object detection on a combination between the depth frame and the IR frame using the MaskR-CNN object detection example from OpenCV and the Aditof SDK.

MaskR-CNN needs access to 3 configuration files: COCO object class LABELS (**object_detection_classes_coco.txt**) and TENSORFLOW model configuration files, based on COCO dataset (**frozen_inference_graph.pb and mask_rcnn_inception_v2_coco_2018_01_28.pbtxt**). For changing the model configuration files use the script:(**https://github.com/opencv/opencv_extra/blob/master/testdata/dnn/download_models.py**).

For running the python program use:
```console
python maskr_cnn.py --model local_path\aditof_sdk\bindings\python\examples\maskr_cnn\frozen_inference_graph.pb --config local_path\aditof_sdk\bindings\python\examples\maskr_cnn\mask_rcnn_inception_v2_coco_2018_01_28.pbtxt --classes local_path\aditof_sdk\bindings\python\examples\maskr_cnn\object_detection_classes_coco.txt
```

![Display Image](/doc/img/maskrcnn_python.png) 
