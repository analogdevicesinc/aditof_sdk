from tkinter import *
from tkinter import ttk
import aditofpython as tof
import numpy as np
import cv2 as cv
from enum import Enum
from process import ProcessTab
from PIL import Image, ImageTk


class ModesEnum(Enum):
    MODE_NEAR = 0
    MODE_MEDIUM = 1
    MODE_FAR = 2


smallSignalThreshold = 100


class GestureDemo(Frame):

    def __init__(self, isapp=True, name='gesturedemo'):
        Frame.__init__(self, name=name)
        self.pack(expand=Y, fill=BOTH)
        self.master.title('Gesture Demo')
        self.isapp = isapp
        self._create_widgets()

    def _create_widgets(self):
        self._create_main_panel()

    def _create_main_panel(self):

        mainPanel = Frame(self, name='demo')
        mainPanel.pack(side=TOP, fill=BOTH, expand=Y)

        # create the notebook
        nb = ttk.Notebook(mainPanel, name='gesturedemo')

        nb.pack(fill=BOTH, expand=Y, padx=2, pady=3)
        self._create_video_tab(nb)
        self._create_process_tab(nb)

    # =============================================================================
    def _create_video_tab(self, nb):
        # frame to hold contentx
        frame = Frame(nb, name='video')

        # widgets to be displayed on Video tab
        msg = ["Capture an image for processing"]
        lbl = Label(frame, justify=LEFT, anchor=N,
                    text=''.join(msg))
        lbl_frame = LabelFrame(frame, bg='red')
        self.lbl_vid = Label(lbl_frame, bg='blue')
        btn = Button(frame, text='Init. Dev', underline=0,
                     command=lambda: self._init_dev())
        btn_start = Button(frame, text='Start Video', underline=0,
                           command=lambda: self._start_video())
        btn_capture = Button(frame, text="Snap!", command=lambda: self._capture_img())

        # position and set resize behaviour
        lbl.grid(row=0, column=0)
        lbl_frame.grid(row=0, column=1, columnspan=4)
        self.lbl_vid.grid(row=0, column=1, columnspan=4)
        btn.grid(row=1, column=0, pady=(2, 4))
        btn_start.grid(row=2, column=0, pady=(2, 4))
        btn_capture.grid(row=3, column=0, pady=(2, 4))
        nb.add(frame, text='Video', padding=2)

    # =============================================================================
    def _init_dev(self):
        system = tof.System()
        print(system)

        self.cameras = []

        status = system.getCameraListAtIp(self.cameras, "10.42.0.18")
        if not status:
            print("system.getCameraListAtIp() failed with status: ", status)

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

        status = self.cameras[0].setFrameType(types[0])
        if not status:
            print("cameras[0].setFrameType() failed with status:", status)

        status = self.cameras[0].setMode(modes[ModesEnum.MODE_NEAR.value])
        if not status:
            print("cameras[0].setMode() failed with status: ", status)

    # =============================================================================
    def _start_video(self):
        camDetails = tof.CameraDetails()
        status = self.cameras[0].getDetails(camDetails)
        if not status:
            print("system.getDetails() failed with status: ", status)

        # Enable noise reduction for better results
        smallSignalThreshold = 100
        self.cameras[0].setControl("noise_reduction_threshold", str(smallSignalThreshold))

        camera_range = camDetails.depthParameters.maxDepth
        distance_scale = 255.0 / camera_range
        tof_frame = tof.Frame()
        while True:
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
            depth_map = cv.applyColorMap(depth_map, cv.COLORMAP_RAINBOW)
            depth_map = cv.cvtColor(depth_map, cv.COLOR_BGR2RGB)
            self.img_obj = Image.fromarray(depth_map)
            img = ImageTk.PhotoImage(self.img_obj.resize((640, 480)))
            self.lbl_vid['image'] = img
            self.update()

    # =============================================================================
    def _capture_img(self):
        file_name = "test.png"
        self.img_obj.resize((640, 480)).save(file_name)

    # =============================================================================
    def _create_process_tab(self, nb):
        # add the process tab to main panel
        frame = ProcessTab(nb)
        nb.add(frame, text='Process', underline=0)


if __name__ == '__main__':
    GestureDemo().mainloop()
