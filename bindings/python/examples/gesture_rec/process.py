import numpy as np
from skimage import measure
from scipy.spatial import ConvexHull
from scipy.ndimage import distance_transform_edt

# These are empirical values, which were found by trial and error
pixel_no_threshold = 200
analyzed_region_distance = 38.25
hand_radius_error = 1.9


class ProcessTab:

    def __init__(self, img_obj):
        self.stop_distance = 0.3
        self.depth_img = img_obj
        self.resultVar = "How many fingers?"
        self._display_result()

    # =============================================================================
    def _display_result(self):
        self._depth_img_hist()
        self._detect_hand()
        self._count_fingers()

    # =============================================================================
    def _depth_img_hist(self):
        # hist[0] = number of elements
        # hist[1] = distance normalized to 255 (this was done in notebook.py)
        counts, bins = np.histogram(self.depth_img.ravel(), 512)
        hist = [counts, bins]

        start = 1
        while start < (len(hist[0]) - 2) and hist[0][start] < pixel_no_threshold:
            start += 1
        bin_step = hist[1][start + 1] - hist[1][start]

        stop = int((hist[1][start] + analyzed_region_distance - hist[1][0]) / bin_step)

        self.stop_distance =  hist[1][stop]
        # Analyze the pixel only if it is closer than a threshold
        self.binary_img = (self.depth_img < self.stop_distance) * 1

    # =============================================================================
    def _detect_hand(self):
        # Find all the objects in the image and select the largest one, which is the hand
        labels = measure.label(self.binary_img)
        props = measure.regionprops(labels)
        if len(props) == 0:
            raise Exception("No object found.")

        props.sort(key=lambda x: x.area, reverse=True)
        self.max_area = props[0]
        points = props[0].filled_image

        # points_hull[0] = x_coordinate; points_hull[1] = y_coordinate
        points_hull = np.where(points == 1)
        self.cord = list(zip(points_hull[0], points_hull[1]))

        # Compute the distance of non-zero points (hand) to the nearest zero point (background)
        self.dist_map = distance_transform_edt(points)
        # Indices of hand center, i.e. the point farthest from the background
        self.hand_center = tuple(arr[0] for arr in np.where(self.dist_map == np.max(self.dist_map)))
        self.radius = hand_radius_error * np.max(self.dist_map)

    # =============================================================================
    def _count_fingers(self):
        # Find the convex hull = contour of the hand with extremities
        hull = ConvexHull(self.cord)
        cord_arr = np.array(self.cord)
        vertices = cord_arr[hull.vertices]

        # delta_x and delta_y bw two consecutive vertices
        dist = np.append(vertices[0:len(vertices) - 1] - vertices[1:len(vertices)],
                         [vertices[-1] - vertices[0]], axis=0)

        # distance bw 2 consecutive vertices
        # In cdist variables the distance units are pixels
        cdist = np.sqrt(dist[:, 0] ** 2 + dist[:, 1] ** 2)

        # TODO: Use a better formula
        # It is used to make cdist_threshold inversely proportional to stop_distance,
        # while keeping it between 0 and 25
        cdist_threshold = np.sqrt(1 - self.stop_distance / 255) * 25
        cdist_bin = (cdist <= cdist_threshold) * 1

        # Used to check whether a cdist smaller than the threshold
        # is following a cdist bigger than the threshold
        cdist_diff = np.append(cdist_bin[0:len(cdist_bin) - 1] - cdist_bin[1:len(cdist_bin)],
                               [cdist_bin[-1] - cdist_bin[0]], axis=0)

        # Indices of vertices which correspond to fingertips
        dist_idx = np.where((cdist_diff == -1) | ((cdist_diff == 0) & (cdist_bin == 0)))
        dist_idx = np.array(dist_idx) + 1
        # dist_idx is a double list
        dist_idx = dist_idx[0]
        if dist_idx[-1] == len(vertices):
            dist_idx[-1] = 0
        # Put 0 in front
        dist_idx.sort()

        # From vertices close to each other select the one which is farthest from the center
        # Compute the possible fingertips distances to the center of the hand
        finger_cdist = [0] * len(dist_idx)
        for i in range(len(dist_idx)):
            stop = dist_idx[i+1] if (i+1 < len(dist_idx)) else len(vertices)
            for j in range(dist_idx[i], stop):
                vertex_dist_from_center = vertices[j] - self.hand_center
                vertex_cdist_from_center = np.sqrt(vertex_dist_from_center[0] ** 2 +
                                                   vertex_dist_from_center[1] ** 2)
                if finger_cdist[i] < vertex_cdist_from_center:
                    finger_cdist[i] = vertex_cdist_from_center
                    dist_idx[i] = j

        # Select actual fingertips
        self.fingers = np.where(np.logical_and(finger_cdist > self.radius,
                                vertices[dist_idx, 0] < (self.hand_center[0] + self.radius)))

        self.detect_gesture()

    # =============================================================================
    def detect_gesture(self):
        if len(self.fingers[0]) == 5:
            self.resultVar = "Found 5 extended fingers.Paper"
        elif len(self.fingers[0]) == 2 or len(self.fingers[0]) == 3:
            self.resultVar = "Found " + str(len(self.fingers[0])) + " extended fingers. Scissors"
        elif len(self.fingers[0]) == 0:
            self.resultVar = "Found " + str(len(self.fingers[0])) + " extended fingers. Rock"
        else:
            self.resultVar = "Found " + str(len(self.fingers[0])) + " extended fingers. Unknown gesture"

