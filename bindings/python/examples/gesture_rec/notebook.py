
from tkinter import *
from tkinter import ttk
import aditofpython as tof
import time
import concurrent.futures
import numpy as np
import cv2 as cv
from enum import Enum
from process import ProcessTab

smallSignalThreshold = 100


class ModesEnum(Enum):
    MODE_NEAR = 0
    MODE_MEDIUM = 1
    MODE_FAR = 2


def calc_process(depth_map):
    process = ProcessTab(depth_map)
    bounding_box = process.max_area.bbox

    return process.resultVar, (bounding_box[1], bounding_box[0]), (bounding_box[3], bounding_box[2])


class GestureDemo(Frame):

    def __init__(self, name='gesturedemo'):
        Frame.__init__(self, name=name)
        self.pack(expand=Y, fill=BOTH)
        self.master.title('Gesture Demo')
        self.resultVar = StringVar()
        self.box_start_point = (0, 0)
        self.box_end_point = (0, 0)
        self._create_main_panel()

    def _create_main_panel(self):

        main_panel = Frame(self, name='demo')
        main_panel.pack(side=TOP, fill=BOTH, expand=Y)

        # create the notebook
        nb = ttk.Notebook(main_panel, name='gesturedemo')

        nb.pack(fill=BOTH, expand=Y, padx=2, pady=3)

        self._create_video_tab(nb)

    # =============================================================================
    def _create_video_tab(self, nb):
        # frame to hold content
        frame = Frame(nb, name='video')
        # widgets to be displayed on Video tab
        msg = ["Capture an image for processing"]
        lbl = Label(frame, justify=LEFT, anchor=N,
                    text=''.join(msg))
        lbl_frame = LabelFrame(frame, bg='red')
        btn = Button(frame, text='Init. Dev', underline=0,
                     command=lambda: self._init_dev())
        btn_start = Button(frame, text='Start Video', underline=0,
                           command=lambda: self._start_video())

        # position and set resize behaviour
        lbl.grid(row=0, column=0)
        lbl_frame.grid(row=0, column=1, columnspan=4)
        btn.grid(row=1, column=0, pady=(2, 4))
        btn_start.grid(row=2, column=0, pady=(2, 4))

        self.resultVar.set("How many fingers?")
        lbl_result = Label(frame, textvariable=self.resultVar, name='result')
        lbl_result.grid(row=3, column=0)

        nb.add(frame, text='Video', padding=2)

    # =============================================================================
    def _init_dev(self):
        system = tof.System()
        print(system)

        self.cameras = []

        status = system.getCameraList(self.cameras)
        if not status:
            print("system.getCameraList failed with status: ", status)

        status = self.cameras[0].initialize()
        if not status:
            print("cameras[0].initialize() failed with status: ", status)

        modes = []
        status = self.cameras[0].getAvailableModes(modes)
        if not status:
            print("system.getAvailableModes() failed with status: ", status)

        types = []
        status = self.cameras[0].getAvailableFrameTypes(types)
        if not status:
            print("system.getAvailableFrameTypes() failed with status: ", status)

        # Use only depth image for faster conversion
        status = self.cameras[0].setFrameType(types[1])
        if not status:
            print("cameras[0].setFrameType() failed with status:", status)

        status = self.cameras[0].setMode(modes[ModesEnum.MODE_NEAR.value])
        if not status:
            print("cameras[0].setMode() failed with status: ", status)

    # =============================================================================
    def _start_video(self):
        cam_details = tof.CameraDetails()
        status = self.cameras[0].getDetails(cam_details)
        if not status:
            print("system.getDetails() failed with status: ", status)

        # Enable noise reduction for better results
        self.cameras[0].setControl("noise_reduction_threshold", str(smallSignalThreshold))

        camera_range = cam_details.depthParameters.maxDepth
        distance_scale = 255.0 / camera_range
        tof_frame = tof.Frame()
        computation_delay_start = time.time()
        process_results = []
        executor = concurrent.futures.ProcessPoolExecutor()

        while True:
            if cv.waitKey(1) != 255: break

            # Capture frame-by-frame
            status = self.cameras[0].requestFrame(tof_frame)
            if not status:
                print("cameras[0].requestFrame() failed with status: ", status)

            depth_map = np.array(tof_frame.getData(tof.FrameDataType.Depth), dtype="uint16", copy=False)
            # Creation of the Depth image
            depth_map = depth_map[0: 480, 0:640]
            depth_map = cv.flip(depth_map, 1)
            depth_map = distance_scale * depth_map
            depth_map = np.uint8(depth_map)

            # Image to display
            img = cv.applyColorMap(depth_map, cv.COLORMAP_RAINBOW)
            cv.rectangle(img, self.box_start_point, self.box_end_point, (0, 255, 0), 15)
            cv.putText(img, self.resultVar.get(), tuple(coord + 5 for coord in self.box_start_point),
                       cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255))
            cv.namedWindow('Depth image', cv.WINDOW_AUTOSIZE)
            cv.imshow('Depth image', img)

            # if process_results != []:
            process_results = self.update_display(process_results)

            # Process image every 1s
            if (time.time() - computation_delay_start) <= 1: continue

            p = executor.submit(calc_process, depth_map)
            process_results.append(p)
            computation_delay_start = time.time()

        executor.shutdown()
        cv.destroyWindow("Depth image")

    def update_display(self, process_results):
        to_delete = []
        processes = concurrent.futures.as_completed(process_results)
        for p in processes:
            try:
                result, self.box_start_point, self.box_end_point = p.result()
                self.resultVar.set(result)
            except Exception as e:
                self.resultVar.set("None")
                print("Exception:", e)
            finally:
                self.update()
                to_delete.append(p)

        return [p for p in process_results if p not in to_delete]


if __name__ == '__main__':
    GestureDemo().mainloop()
