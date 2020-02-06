import aditofpython as tof
import numpy as np
import cv2 as cv
import open3d as o3d
from enum import Enum

WINDOW_NAME_DEPTH = "Display Depth"
WINDOW_NAME_COLOR = "Display Color"


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

    specifics = cameras[0].getCamera96Tof1Specifics()
    specifics.setCameraRevision(tof.Revision.RevC)

    status = cameras[0].setMode(modes[ModesEnum.MODE_NEAR.value])
    if not status:
        print("cameras[0].setMode() failed with status: ", status)

    camDetails = tof.CameraDetails()
    status = cameras[0].getDetails(camDetails)
    if not status:
        print("system.getDetails() failed with status: ", status)

    # Enable noise reduction for better results
    smallSignalThreshold = 100
    specifics.setNoiseReductionThreshold(smallSignalThreshold)
    specifics.enableNoiseReduction(True)

    # Get intrinsic parameters from camera
    intrinsicParameters = camDetails.intrinsics
    fx = intrinsicParameters.cameraMatrix[0]
    fy = intrinsicParameters.cameraMatrix[4]
    cx = intrinsicParameters.cameraMatrix[2]
    cy = intrinsicParameters.cameraMatrix[5]
    width = 640
    height = 480
    cameraIntrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

    camera_range = camDetails.maxDepth
    bitCount = camDetails.bitCount
    frame = tof.Frame()

    max_value_of_IR_pixel = 2 ** bitCount - 1
    distance_scale_ir = 255.0 / max_value_of_IR_pixel
    distance_scale = 255.0 / camera_range

    while True:
        # Capture frame-by-frame
        status = cameras[0].requestFrame(frame)
        if not status:
            print("cameras[0].requestFrame() failed with status: ", status)

        depth_map = np.array(frame.getData(tof.FrameDataType.Depth), dtype="uint16", copy=False)
        ir_map = np.array(frame.getData(tof.FrameDataType.IR), dtype="uint16", copy=False)

        # Creation of the IR image
        ir_map = ir_map[0: int(ir_map.shape[0] / 2), :]
        ir_map = distance_scale_ir * ir_map
        ir_map = np.uint8(ir_map)
        ir_map = cv.cvtColor(ir_map, cv.COLOR_GRAY2RGB)

        # Creation of the Depth image
        new_shape = (int(depth_map.shape[0] / 2), depth_map.shape[1])
        depth16bits_map = depth_map = np.resize(depth_map, new_shape)
        depth_map = distance_scale * depth_map
        depth_map = np.uint8(depth_map)
        depth_map = cv.applyColorMap(depth_map, cv.COLORMAP_RAINBOW)

        # Show Depth map :
        cv.namedWindow(WINDOW_NAME_DEPTH, cv.WINDOW_AUTOSIZE)
        cv.imshow(WINDOW_NAME_DEPTH, depth_map)

        img_color = cv.addWeighted(ir_map, 0.4, depth_map, 0.6, 0)

        # Show Depth+IR combined map
        cv.namedWindow(WINDOW_NAME_COLOR, cv.WINDOW_AUTOSIZE)
        cv.imshow(WINDOW_NAME_COLOR, img_color)

        color_raw = o3d.geometry.Image(img_color)
        depth16bits_raw = o3d.geometry.Image(depth16bits_map)

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth16bits_raw, 1000.0, 3.0, False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, cameraIntrinsics)

        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        o3d.visualization.draw_geometries([pcd])

        if cv.waitKey(1) >= 0:
            break
