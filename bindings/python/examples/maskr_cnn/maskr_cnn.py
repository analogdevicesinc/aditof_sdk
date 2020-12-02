#
# BSD 3-Clause License
#
# Copyright (c) 2019, Analog Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
import aditofpython as tof
import cv2 as cv
import argparse
import numpy as np
from enum import Enum
import sys
import threading
import time
import statistics

thr = 0.5


class ModesEnum(Enum):
    MODE_NEAR = 0
    MODE_MEDIUM = 1
    MODE_FAR = 2


WINDOW_MASKRCNN = "Mask-RCNN in OpenCV"
WINDOW_COLOR = "Color image"


def apply_maskrcnn_algorithm(distance_map, image, imageH, imageW):
    start_detection = time.time()
    # Create a 4D blob from a frame.
    blob = cv.dnn.blobFromImage(image, swapRB=True, crop=False)

    # Run a model
    net.setInput(blob)
    boxes, masks = net.forward(['detection_out_final', 'detection_masks'])

    # Generate colors
    colors = [np.array([0, 0, 0], np.uint8)]
    for i in range(1, masks.shape[1] + 1):
        colors.append((colors[i - 1] + np.random.randint(0, 256, [3], np.uint8)) / 2)
    del colors[0]

    for i in range(0, boxes.shape[2]):
        box = boxes[0, 0, i]
        mask = masks[i]
        score = box[2]
        if score > thr:
            classId = int(box[1])
            left = int(imageW * box[3])
            top = int(imageH * box[4])
            right = int(imageW * box[5])
            bottom = int(imageH * box[6])

            left = max(0, min(left, imageW - 1))
            top = max(0, min(top, imageH - 1))
            right = max(0, min(right, imageW - 1))
            bottom = max(0, min(bottom, imageH - 1))

            classMask = mask[classId]
            classMask = cv.resize(classMask, (right - left + 1, bottom - top + 1))
            mask = (classMask > 0.5)

            roi = image[top:bottom + 1, left:right + 1][mask]
            blended = ((0.7 * colors[classId]) + (0.3 * roi)).astype("uint8")
            image[top:bottom + 1, left:right + 1][mask] = blended

            # Calculate the distance to the object
            roi_distance = distance_map[top:bottom + 1, left:right + 1][mask]
            distance = statistics.mean(roi_distance) / 1000.0

            # Draw the bounding box of the instance on the image
            color = [int(c) for c in colors[classId]]
            cv.rectangle(image, (left, top), (right + 1, bottom + 1), color, 1)

            # Draw the predicted label, the distance and associated probability of detection
            text_distance = "{}: {:.4f} meters".format(classes[classId], distance)
            text_distance_size, baseline_distance = cv.getTextSize(text_distance, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            text_conf = "{}: {:.4f}".format("Confidence: ", score)
            text_conf_size, baseline_conf = cv.getTextSize(text_conf, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)

            if text_distance_size[0] > text_conf_size[0]:
                label_size = text_distance_size
                label_baseline = baseline_distance
            else:
                label_size = text_conf_size
                label_baseline = baseline_conf

            cv.rectangle(image, ((int((left + right) * 0.5) - int(label_size[0] * 0.5)),
                                 top + label_baseline),
                         ((int((left + right) * 0.5) + int(label_size[0] * 0.5)),
                          (top + 2 * label_size[1] + 3 * label_baseline)),
                         (255, 255, 255), cv.FILLED)
            cv.putText(image, text_distance, ((int((left + right) * 0.5) - int(label_size[0] * 0.5)),
                                              top + label_size[1] + label_baseline),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv.putText(image, text_conf, ((int((left + right) * 0.5) - int(label_size[0] * 0.5)),
                                          top + 2 * label_baseline + 2 * label_size[1]),
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # Calculate the time for computing the algorithm
    end_detection = time.time()
    text_detect = "Time per frame: {:.2f}s".format(end_detection - start_detection)
    text_size, baseline_text = cv.getTextSize(text_detect, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv.rectangle(image, (10, imageH - 10 - text_size[1] - int(baseline_text * 0.5)),
                 (10 + text_size[0], imageH - 10 + baseline_text),
                 (255, 255, 255), cv.FILLED)
    cv.putText(image, text_detect, (10, imageH - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    # Show image
    cv.imshow(WINDOW_MASKRCNN, image)
    cv.waitKey(1)

def get_scaling_values(cameraDetails):
    camera_range = cameraDetails.maxDepth
    bit_count = cameraDetails.bitCount
    max_value_of_IR_pixel = 2 ** bit_count - 1
    distance_scale_ir = 255.0 / max_value_of_IR_pixel
    distance_scale_depth = 255.0 / camera_range
    return distance_scale_ir, distance_scale_depth


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description= 'Script to run Mask-RCNN object detection')
    parser.add_argument('--model', required=True, help='Path to a .pb file with weights.')
    parser.add_argument('--config', required=True, help='Path to a .pxtxt file contains network configuration.')
    parser.add_argument('--classes', required=True, help='Path to a text file with names of classes.')
    args = parser.parse_args()

    # Used for colors generation
    np.random.seed(324)

    # Load names of classes
    classes = None
    with open(args.classes, 'rt') as f:
        classes = f.read().rstrip('\n').split('\n')

    # Load a network
    try:
        net = cv.dnn.readNetFromTensorflow(cv.samples.findFile(args.model), cv.samples.findFile(args.config))
    except:
        print("Error: Please give the correct location of the model and configuration")
        sys.exit(1)
    # net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)

    # Initialize ToF camera
    system = tof.System()

    cameras = []
    status = system.getCameraList(cameras)
    if not status:
        print("system.getCameraList() failed with status: ", status)

    modes = []
    status = cameras[0].getAvailableModes(modes)
    if not status:
        print("system.getAvailableModes() failed with status: ", status)

    types = []
    status = cameras[0].getAvailableFrameTypes(types)
    if not status:
        print("system.getAvailableFrameTypes() failed with status: ", status)

    status = cameras[0].initialize()
    if not status:
        print("cameras[0].initialize() failed with status: ", status)

    status = cameras[0].setFrameType(types[0])
    if not status:
        print("cameras[0].setFrameType() failed with status:", status)

    status = cameras[0].setMode(modes[ModesEnum.MODE_NEAR.value])
    if not status:
        print("cameras[0].setMode() failed with status: ", status)

    camDetails = tof.CameraDetails()
    status = cameras[0].getDetails(camDetails)
    if not status:
        print("system.getDetails() failed with status: ", status)

    # Enable noise reduction for better results
    smallSignalThreshold = 100
    cameras[0].setControl("noise_reduction_threshold", str(smallSignalThreshold))

    # Get the scaling values for depth and ir image
    distance_scale_ir, distance_scale_depth = get_scaling_values(camDetails)

    cv.namedWindow(WINDOW_MASKRCNN, cv.WINDOW_AUTOSIZE)
    cv.namedWindow(WINDOW_COLOR, cv.WINDOW_AUTOSIZE)
    start = time.time()
    start_thread = 1
    displayFps = 0
    frameCount = 0
    frame = tof.Frame()

    while True:
        # Capture frame-by-frame
        frameCount = frameCount + 1
        status = cameras[0].requestFrame(frame)
        if not status:
            print("cameras[0].requestFrame() failed with status: ", status)

        depth_map = np.array(frame.getData(tof.FrameDataType.Depth), dtype="uint16", copy=False)
        ir_map = np.array(frame.getData(tof.FrameDataType.IR), dtype="uint16", copy=False)

        # Creation of the IR image
        ir_map = ir_map[0: int(ir_map.shape[0] / 2), :]
        ir_map = distance_scale_ir * ir_map
        ir_map = np.uint8(ir_map)
        ir_map = cv.flip(ir_map, 1)
        ir_map = cv.cvtColor(ir_map, cv.COLOR_GRAY2RGB)

        # Creation of the Depth image
        depth_map = depth_map[0: int(depth_map.shape[0] / 2), :]
        depth_map = cv.flip(depth_map, 1)
        distance_map = depth_map
        depth_map = distance_scale_depth * depth_map
        depth_map = np.uint8(depth_map)
        depth_map = cv.applyColorMap(depth_map, cv.COLORMAP_RAINBOW)

        # Combine depth and IR for more accurate results
        color_map = cv.addWeighted(ir_map, 0.4, depth_map, 0.6, 0)
        show_image = cv.addWeighted(ir_map, 0.4, depth_map, 0.6, 0)

        width = color_map.shape[1]
        height = color_map.shape[0]

        # Calculate and display FPS for color image
        end = time.time()
        elap = (end - start)
        if elap >= 2:
            displayFps = frameCount / elap
            frameCount = 0
            start = end
        fps_text = "FPS: {:.2f}".format(int(displayFps))
        fps_text_size, baseline = cv.getTextSize(fps_text, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv.rectangle(show_image, (10, height - 10 - fps_text_size[1] - int(baseline * 0.5)),
                     (10 + fps_text_size[0], height - 10 + baseline),
                     (255, 255, 255), cv.FILLED)
        cv.putText(show_image, fps_text, (10, height - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # Show the image
        cv.imshow(WINDOW_COLOR, show_image)

        if start_thread == 1:
            thread_maskrcnn = threading.Thread(target=apply_maskrcnn_algorithm,
                                               args=(distance_map, color_map, height, width))
            thread_maskrcnn.start()
        if thread_maskrcnn.is_alive():
            start_thread = 0
        else:
            start_thread = 1

        # Wait for user input
        if cv.waitKey(1) >= 0:
            break
