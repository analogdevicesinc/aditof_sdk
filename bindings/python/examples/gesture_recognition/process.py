from tkinter import filedialog
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from tkinter import *
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from skimage.color import rgb2hsv
from skimage import measure
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from scipy.ndimage import distance_transform_edt
from scipy import ndimage

hue_threshold = 0.3


class GestureRecognitionApp(Frame):

    def __init__(self, isapp=True, name='gesture_recognition'):
        Frame.__init__(self, name=name)
        self.pack(expand=Y, fill=BOTH)
        self.master.title('Gesture Demo')
        self.isapp = isapp
        self._create_widgets()

    def _create_widgets(self):
        self._add_canvases()
        btn_load = Button(self, text='Load',
                          command=lambda: self._load_img())
        self.resultVar = StringVar()
        self.resultVar.set("How many fingers?")
        btn_process = Button(self, text='Process',
                             command=lambda v=self.resultVar: self._display_result(v))
        lbl_result = Label(self, textvariable=self.resultVar, name='result')
        btn_load.grid(row=0, column=2, pady=(2, 4))
        btn_process.grid(row=1, column=2, pady=(2, 4))
        lbl_result.grid(row=2, column=2)

    def _add_canvases(self):
        # input image canvas
        fig_input, self.ax_input = plt.subplots(figsize=(4, 3))
        self.ax_input.set_title("Depth image")
        self.input_canvas = FigureCanvasTkAgg(fig_input, master=self)
        self.input_canvas.get_tk_widget().grid(row=1, column=0)
        # result image canvas
        fig_result, self.ax_result = plt.subplots(figsize=(4, 3))
        self.ax_result.set_title("Detected Hand and Fingers")
        self.result_canvas = FigureCanvasTkAgg(fig_result, master=self)
        self.result_canvas.get_tk_widget().grid(row=1, column=1)
        # process plots canvas
        fig, (self.ax1, self.ax2) = plt.subplots(ncols=2, figsize=(8, 3))
        self.ax_settings()
        self.canvas = FigureCanvasTkAgg(fig, master=self)
        self.canvas.get_tk_widget().grid(row=2, column=0, columnspan=2)

    # =============================================================================
    def _load_img(self):
        filename = filedialog.askopenfilename(
            title='Open image',
            initialdir=os.getcwd())
        self.ax_input.clear()
        self.input_canvas.draw()
        self.rgb_img = mpimg.imread(filename)
        self.ax_input.imshow(self.rgb_img)
        self.input_canvas.draw()

    # =============================================================================
    def _display_result(self, v):
        self._hue_ch_hist()
        self._detect_hand()
        self._count_fingers()

    # =============================================================================
    def _hue_ch_hist(self):
        self.ax1.clear()
        self.ax2.clear()
        self.ax_result.clear()
        self.result_canvas.draw()
        # Remove the background
        hsv_img = rgb2hsv(self.rgb_img)
        hue_img = hsv_img[:, :, 0]
        self.binary_img = (hue_img < hue_threshold) * 1
        self.ax1.hist(hue_img.ravel(), 512)
        self.ax2.imshow(self.binary_img, cmap='gray')
        self.ax_settings()
        self.canvas.draw()

    # =============================================================================
    def _detect_hand(self):
        # Find all the objects in the image and select the largest one, which is the hand
        labels = measure.label(self.binary_img)
        props = measure.regionprops(labels)
        props.reverse()
        max_area = props[0]
        points = props[0].filled_image
        points_hull = np.where(points == 1)
        self.cord = list(zip(points_hull[0], points_hull[1]))
        self.dist_map = distance_transform_edt(points)
        self.hand_center = ndimage.center_of_mass(self.dist_map)
        self.radius = 1.75 * np.max(self.dist_map)

    # =============================================================================
    def _count_fingers(self):
        # Find the convex hull = contour of the hand with extremities
        hull = ConvexHull(self.cord)
        cord_arr = np.array(self.cord)
        vertices = cord_arr[hull.vertices]
        vertices = vertices[1:len(vertices) - 2]

        dist = vertices[0:len(vertices) - 1] - vertices[1:len(vertices)]
        cdist = np.sqrt(dist[:, 0] ** 2 + dist[:, 1] ** 2)
        cdist = (cdist < 15) * 1
        cdist = cdist[0:len(cdist) - 1] - cdist[1:len(cdist)]
        if cdist[0] != 0:
            cdist[0] = -1
        if cdist[len(cdist) - 1] != 0:
            cdist[len(cdist) - 1] = -1
        dist_idx = np.where(cdist == -1)
        dist_idx = np.array(dist_idx) + 1

        # Compute the finger tips distances to the center of the hand and detect the gesture
        finger_dist = np.array(vertices[dist_idx]) - self.hand_center
        for fd in finger_dist:
            finger_cdist = np.sqrt(fd[:, 0] ** 2 + fd[:, 1] ** 2)
        self.fingers = np.where(finger_cdist > self.radius)
        self.detect_gesture()

        self.ax_result.set_title("Detected hand and fingers")
        self.ax_result.imshow(self.dist_map, cmap='gray')
        self.ax_result.plot(self.hand_center[1], self.hand_center[0], "o", mec='g', color='none', markersize=self.radius)
        self.ax_result.plot(vertices[dist_idx, 1], vertices[dist_idx, 0], 'o', mec='r', color='none', lw=1, markersize=10)
        self.ax_result.plot(vertices[:, 1], vertices[:, 0], 'x', mec='g', color='none', lw=1, markersize=4)
        self.result_canvas.draw()

    # =============================================================================
    def detect_gesture(self):

        if len(self.fingers[0]) == 5:
            self.resultVar.set("Found " + str(len(self.fingers[0])) + " extended fingers.Paper")
        elif len(self.fingers[0]) == 2 or len(self.fingers[0]) == 3:
            self.resultVar.set("Found " + str(len(self.fingers[0])) + " extended fingers. Scissors")
        elif len(self.fingers[0]) == 0:
            self.resultVar.set("Found " + str(len(self.fingers[0])) + " extended fingers. Rock")
        else:
            self.resultVar.set("Found " + str(len(self.fingers[0])) + " extended fingers. Unknown gesture")

    # =============================================================================
    def ax_settings(self):
        self.ax1.set_title("Histogram of the Hue channel with threshold")
        self.ax1.set_ylim([0, 3000])
        self.ax1.axvline(x=hue_threshold, color='r', linestyle='dashed', linewidth=2)
        self.ax2.set_title("Hue-thresholded image")
        self.ax2.axis('off')


if __name__ == '__main__':
    GestureRecognitionApp().mainloop()
