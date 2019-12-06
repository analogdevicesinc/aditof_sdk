import aditofpython as tof
import numpy as np
import cv2 as cv
import argparse
from enum import Enum
import sys

inWidth = 300
inHeight = 300
WHRatio = inWidth / float(inHeight)
inScaleFactor = 0.007843
meanVal = 127.5
thr = 0.2
WINDOW_NAME = "Display Objects"
WINDOW_NAME_DEPTH = "Display Objects Depth"


class ModesEnum(Enum):
    MODE_NEAR = 0
    MODE_MEDIUM = 1
    MODE_FAR = 2


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Script to run MobileNet-SSD object detection network ')
    parser.add_argument("--prototxt", default="MobileNetSSD_deploy.prototxt",
                        help='Path to text network file: '
                             'MobileNetSSD_deploy.prototxt')
    parser.add_argument("--weights", default="MobileNetSSD_deploy.caffemodel",
                        help='Path to weights: '
                             'MobileNetSSD_deploy.caffemodel')
    args = parser.parse_args()
    try:
        net = cv.dnn.readNetFromCaffe(args.prototxt, args.weights)
    except:
        print("Error: Please give the correct location of the prototxt and caffemodel")
        sys.exit(1)

    swapRB = False
    classNames = {0: 'background',
                  1: 'aeroplane', 2: 'bicycle', 3: 'bird', 4: 'boat',
                  5: 'bottle', 6: 'bus', 7: 'car', 8: 'cat', 9: 'chair',
                  10: 'cow', 11: 'diningtable', 12: 'dog', 13: 'horse',
                  14: 'motorbike', 15: 'person', 16: 'pottedplant',
                  17: 'sheep', 18: 'sofa', 19: 'train', 20: 'tvmonitor'}

    system = tof.System()
    status = system.initialize()
    if not status:
        print("system.initialize() failed with status: ", status)

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

    status = cameras[0].setMode(modes[ModesEnum.MODE_MEDIUM.value])
    if not status:
        print("cameras[0].setMode() failed with status: ", status)

    camDetails = tof.CameraDetails()
    status = cameras[0].getDetails(camDetails)
    if not status:
        print("system.getDetails() failed with status: ", status)

    device = cameras[0].getDevice()
    smallSignalThreshold = 50;
    REGS_CNT = 5
    afeRegsAddr = [0x4001, 0x7c22, 0xc34a, 0x4001, 0x7c22]
    afeRegsVal = [0x0006, 0x0004, 0x803C, 0x0007, 0x0004]
    device.writeAfeRegisters(afeRegsAddr, afeRegsVal, REGS_CNT)

    camera_range = camDetails.range
    frame = tof.Frame()

    while True:
        # Capture frame-by-frame
        status = cameras[0].requestFrame(frame)
        if not status:
            print("cameras[0].requestFrame() failed with status: ", status)

        depth_map = np.array(frame.getData(tof.FrameDataType.Depth), dtype="uint16", copy=False)
        ir_map = np.array(frame.getData(tof.FrameDataType.IR), dtype="uint16", copy=False)

        # Creation of the IR image
        ir_map = ir_map[0: int(ir_map.shape[0] / 2), :]
        ir_map = np.float32(ir_map)
        distance_scale_ir = 255.0 / camera_range
        ir_map = distance_scale_ir * ir_map
        ir_map = np.uint8(ir_map)
        ir_map = cv.cvtColor(ir_map, cv.COLOR_GRAY2RGB)

        # Creation of the Depth image
        new_shape = (int(depth_map.shape[0] / 2), depth_map.shape[1])
        depth_map = np.resize(depth_map, new_shape)
        distance_map = depth_map
        depth_map = np.float32(depth_map)
        distance_scale = 255.0 / camera_range
        depth_map = distance_scale * depth_map
        depth_map = np.uint8(depth_map)
        depth_map = cv.applyColorMap(depth_map, cv.COLORMAP_RAINBOW)

        # Combine depth and IR for more accurate results
        result = cv.addWeighted(ir_map, 0.4, depth_map, 0.6, 0)

        # Start the computations for object detection using DNN
        blob = cv.dnn.blobFromImage(result, inScaleFactor, (inWidth, inHeight), (meanVal, meanVal, meanVal), swapRB)
        net.setInput(blob)
        detections = net.forward()

        cols = result.shape[1]
        rows = result.shape[0]

        if cols / float(rows) > WHRatio:
            cropSize = (int(rows * WHRatio), rows)
        else:
            cropSize = (cols, int(cols / WHRatio))

        y1 = int((rows - cropSize[1]) / 2)
        y2 = y1 + cropSize[1]
        x1 = int((cols - cropSize[0]) / 2)
        x2 = x1 + cropSize[0]
        result = result[y1:y2, x1:x2]
        depth_map = depth_map[y1:y2, x1:x2]
        distance_map = distance_map[y1:y2, x1:x2]

        cols = result.shape[1]
        rows = result.shape[0]

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > thr:
                class_id = int(detections[0, 0, i, 1])
                xLeftBottom = int(detections[0, 0, i, 3] * cols)
                yLeftBottom = int(detections[0, 0, i, 4] * rows)
                xRightTop = int(detections[0, 0, i, 5] * cols)
                yRightTop = int(detections[0, 0, i, 6] * rows)

                cv.rectangle(result, (xLeftBottom, yLeftBottom), (xRightTop, yRightTop),
                             (0, 255, 0))
                cv.rectangle(depth_map, (xLeftBottom, yLeftBottom), (xRightTop, yRightTop),
                             (0, 255, 0))
                center = ((xLeftBottom + xRightTop) * 0.5, (yLeftBottom + yRightTop) * 0.5)
                if class_id in classNames:
                    value_x = int(center[0])
                    value_y = int(center[1])
                    label = classNames[class_id] + ": " + \
                            "{0:.3f}".format(distance_map[value_x, value_y] / 1000.0 * 0.3) + " " + "meters"
                    labelSize, baseLine = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)

                    yLeftBottom = max(yLeftBottom, labelSize[1])
                    cv.rectangle(result, (value_x - int(labelSize[0] * 0.5), value_y - labelSize[1]),
                                 (value_x + int(labelSize[0] * 0.5), value_y + baseLine),
                                 (255, 255, 255), cv.FILLED)
                    cv.putText(result, label, (value_x - int(labelSize[0] * 0.5), value_y),
                               cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

                    cv.rectangle(depth_map, (value_x - int(labelSize[0] * 0.5), value_y - labelSize[1]),
                                 (value_x + int(labelSize[0] * 0.5), value_y + baseLine),
                                 (255, 255, 255), cv.FILLED)
                    cv.putText(depth_map, label, (value_x - int(labelSize[0] * 0.5), value_y),
                               cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

        # Show image with object detection
        cv.namedWindow(WINDOW_NAME, cv.WINDOW_AUTOSIZE)
        cv.imshow(WINDOW_NAME, result)

        # Show Depth map
        cv.namedWindow(WINDOW_NAME_DEPTH, cv.WINDOW_AUTOSIZE)
        cv.imshow(WINDOW_NAME_DEPTH, depth_map)

        if cv.waitKey(1) >= 0:
            break
