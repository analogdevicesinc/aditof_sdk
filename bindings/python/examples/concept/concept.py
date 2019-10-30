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
WINDOW_NAME_DEPTH = "Display Combined"
WINDOW_NAME_IR = "Display IR"


class ModesEnum(Enum):
    MODE_NEAR = 0
    MODE_MEDIUM = 1
    MODE_FAR = 2


if __name__ == "__main__":

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

    camera_range = camDetails.range
    print(camera_range)
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

        # Show image with object detection
        cv.namedWindow(WINDOW_NAME, cv.WINDOW_AUTOSIZE)
        cv.imshow(WINDOW_NAME, result)

        img = result

        # Show Depth map
        cv.namedWindow(WINDOW_NAME_DEPTH, cv.WINDOW_AUTOSIZE)
        cv.imshow(WINDOW_NAME_DEPTH, depth_map)

        # Show IR map
        cv.namedWindow(WINDOW_NAME_IR, cv.WINDOW_AUTOSIZE)
        cv.imshow(WINDOW_NAME_IR, ir_map)

        imgray = cv.cvtColor(ir_map, cv.COLOR_BGR2GRAY)
        blur = cv.blur(imgray, (3, 3))
        cv.namedWindow("blur", cv.WINDOW_AUTOSIZE)
        cv.imshow("blur", blur)

        edges = cv.Canny(blur, 20, 100)
        cv.namedWindow("Canny", cv.WINDOW_AUTOSIZE)
        cv.imshow("Canny", edges)

        #ret, thresh = cv.threshold(imgray, 127, 255, 0)
        contours, hierarchy = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        cv.drawContours(result, contours, -1, (0, 255, 0), 3)

        print(len(contours))
        cv.namedWindow("contours", cv.WINDOW_AUTOSIZE)
        cv.imshow("contours", result)

        # mask = np.zeros(img.shape[:2], np.uint8)
        # bgdModel = np.zeros((1, 65), np.float64)
        # fgdModel = np.zeros((1, 65), np.float64)
        # rect = (50, 50, 450, 290)
        # cv.grabCut(img, mask, rect, bgdModel, fgdModel, 5, cv.GC_INIT_WITH_RECT)
        # mask2 = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
        # img = img * mask2[:, :, np.newaxis]

        cv.namedWindow("img", cv.WINDOW_AUTOSIZE)
        cv.imshow("img", img)

        if cv.waitKey(1) >= 0:
            break
