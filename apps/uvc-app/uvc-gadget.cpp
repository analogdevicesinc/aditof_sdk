/*
 * UVC gadget test application
 *
 * Copyright (C) 2010 Ideas on board SPRL <laurent.pinchart@ideasonboard.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 */

#include "../../sdk/src/connections/target/v4l_buffer_access_interface.h"
#include "uvc.h"

#include <aditof/depth_sensor_interface.h>
#include <aditof/sensor_enumerator_factory.h>
#include <aditof/sensor_enumerator_interface.h>
#include <aditof/storage_interface.h>
#include <aditof/temperature_sensor_interface.h>
#include <aditof/version.h>
#include <atomic>
#include <errno.h>
#include <fcntl.h>
#include <glog/logging.h>
#include <linux/usb/ch9.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

#define MAX_PACKET_SIZE 60

/* Enable debug prints. */
//#define ENABLE_BUFFER_DEBUG
//#define ENABLE_USB_REQUEST_DEBUG

#ifdef ENABLE_BUFFER_DEBUG
#define LOG_FREQ (30)
#endif

#ifdef ENABLE_USB_REQUEST_DEBUG
#define USB_REQ_DEBUG(X...)                                                    \
    {                                                                          \
        printf("->%s: ", __FUNCTION__);                                        \
        printf(X);                                                             \
    }
#else
#define USB_REQ_DEBUG(X...) //{ printf("->%s: ",__FUNCTION__); printf (X); }
#endif

#define SET_REQ_ERROR_CODE(val, len)                                           \
    ({                                                                         \
        dev->request_error_code.data[0] = val;                                 \
        dev->request_error_code.length = len;                                  \
    })

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define max(a, b) (((a) > (b)) ? (a) : (b))

#define clamp(val, min, max)                                                   \
    ({                                                                         \
        typeof(val) __val = (val);                                             \
        typeof(min) __min = (min);                                             \
        typeof(max) __max = (max);                                             \
        (void)(&__val == &__min);                                              \
        (void)(&__val == &__max);                                              \
        __val = __val < __min ? __min : __val;                                 \
        __val > __max ? __max : __val;                                         \
    })

#define ARRAY_SIZE(a) ((sizeof(a) / sizeof(a[0])))
#define pixfmtstr(x)                                                           \
    (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, ((x) >> 24) & 0xff

/*
 * The UVC webcam gadget kernel driver (g_webcam.ko) supports changing
 * the Brightness attribute of the Processing Unit (PU). by default. If
 * the underlying video capture device supports changing the Brightness
 * attribute of the image being acquired (like the Virtual Video, VIVI
 * driver), then we should route this UVC request to the respective
 * video capture device.
 *
 * Incase, there is no actual video capture device associated with the
 * UVC gadget and we wish to use this application as the final
 * destination of the UVC specific requests then we should return
 * pre-cooked (static) responses to GET_CUR(BRIGHTNESS) and
 * SET_CUR(BRIGHTNESS) commands to keep command verifier test tools like
 * UVC class specific test suite of USBCV, happy.
 *
 * Note that the values taken below are in sync with the VIVI driver and
 * must be changed for your specific video capture device. These values
 * also work well in case there in no actual video capture device.
 */
#define PU_BRIGHTNESS_MIN_VAL 0
#define PU_BRIGHTNESS_MAX_VAL 255
#define PU_BRIGHTNESS_STEP_SIZE 1
#define PU_BRIGHTNESS_DEFAULT_VAL 127

/* ---------------------------------------------------------------------------
 * Generic stuff
 */

/* IO methods supported */
enum io_method {
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
};

/* Buffer representing one video frame */
struct buffer {
    struct v4l2_buffer buf;
    void *start;
    size_t length;
};

/* ---------------------------------------------------------------------------
 * UVC specific stuff
 */

struct uvc_frame_info {
    unsigned int width;
    unsigned int height;
    unsigned int intervals[8];
};

struct uvc_format_info {
    unsigned int fcc;
    const struct uvc_frame_info *frames;
};

static const struct uvc_frame_info uvc_frames_yuyv[] = {
    {
        640,
        960,
        {400000, 1000000, 5000000, 0},
    },
    {
        668,
        750,
        {666666, 1000000, 5000000, 0},
    },
    {
        1280,
        720,
        {1000000, 0},
    },
    {
        640,
        360,
        {666666, 1000000, 5000000, 0},
    },
    {
        0,
        0,
        {0},
    },
};

static const struct uvc_frame_info uvc_frames_mjpeg[] = {
    {
        640,
        360,
        {666666, 10000000, 50000000, 0},
    },
    {
        1280,
        720,
        {50000000, 0},
    },
    {
        0,
        0,
        {
            0,
        },
    },
};

static const struct uvc_format_info uvc_formats[] = {
    {V4L2_PIX_FMT_YUYV, uvc_frames_yuyv},
    {V4L2_PIX_FMT_MJPEG, uvc_frames_mjpeg},
};

/* ---------------------------------------------------------------------------
 * UVC device instance
 */

/* Represents a UVC based video output device */
struct uvc_device {
    /* uvc device specific */
    int uvc_fd;
    int is_streaming;
    int run_standalone;
    const char *uvc_devname;

    /* uvc control request specific */

    struct uvc_streaming_control probe;
    struct uvc_streaming_control commit;
    int control;
    int set_cur_cs;
    struct uvc_request_data request_error_code;
    unsigned int brightness_val;

    /* uvc buffer specific */
    enum io_method io;
    struct buffer *mem;
    struct buffer *dummy_buf;
    unsigned int nbufs;
    unsigned int fcc;
    unsigned int width;
    unsigned int height;

    unsigned int bulk;
    uint8_t color;
    unsigned int imgsize;
    void *imgdata;

    /* USB speed specific */
    int mult;
    int burst;
    int maxpkt;
    enum usb_device_speed speed;

    /* uvc specific flags */
    int first_buffer_queued;
    int uvc_shutdown_requested;

    /* uvc buffer queue and dequeue counters */
    unsigned long long int qbuf_count;
    unsigned long long int dqbuf_count;

    bool hasNewFrame;
};

bool SensorStartedStreaming = false;

int ad903x_hw = 0;
char v4l2_subdev_prog_file[255] = "afe_firmware.bin";

char *sensorsInfoBuffer =
    nullptr; // Points to data holding information about available sensors. First two bytes contain the size of the data in the rest of the buffer

/* TODO: Make this declaration local or part of dev stucture */
struct v4l2_plane gPlanes[8] = {{0}};
int firmware_size = 0, flen = 0;
const static int FIRMWARE_CAPACITY = 16384;
char firmware[FIRMWARE_CAPACITY] = {0};
unsigned short reg_addr;

/* Available sensors */
std::vector<std::shared_ptr<aditof::DepthSensorInterface>> depthSensors;
std::vector<std::shared_ptr<aditof::StorageInterface>> storages;
std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
    temperatureSensors;

/* UVC only works with one depth sensor */
std::shared_ptr<aditof::DepthSensorInterface> camDepthSensor;
std::shared_ptr<aditof::V4lBufferAccessInterface> sensorV4lBufAccess;

/* Storages */
unsigned int eeprom_read_addr = 0;
unsigned int eeprom_read_len = 0;
unsigned int eeprom_write_addr = 0;
unsigned int eeprom_write_len = 0;
unsigned int storage_index = 0;
unsigned char eeprom_data[128 * 1024];
unsigned int sensors_read_pos = 0;
unsigned int sensors_read_len = 0;
std::atomic<bool> eeprom_write_ready;
std::thread eepromWriteThread;

/* Temperature sensors */
unsigned char temp_index = 0;

/* forward declarations */
static int uvc_video_stream(struct uvc_device *dev, int enable);

std::atomic<bool> stop;

void stopHandler(int code) { stop.store(true); }

/* ---------------------------------------------------------------------------
 * V4L2 streaming related
 */

static int v4l2_process_data(uvc_device *udev) {
    int ret;
    struct v4l2_buffer ubuf;
    struct v4l2_buffer vbuf;

    if (!SensorStartedStreaming) {
        return 0;
    }

    /* Queue video buffer to UVC domain. */
    CLEAR(ubuf);

    ubuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

    ubuf.memory = V4L2_MEMORY_USERPTR;

    if (!udev->hasNewFrame) {
        return 0;
    }

    auto status = sensorV4lBufAccess->dequeueInternalBuffer(vbuf);
    ubuf.bytesused = udev->width * udev->height * 2;
    ubuf.index = vbuf.index;
    ubuf.length = vbuf.m.planes[0].length * 2;

    uint8_t *bufferStart;
    uint32_t dummyLength;
    sensorV4lBufAccess->getInternalBuffer(&bufferStart, dummyLength, vbuf);
    ubuf.m.userptr = (unsigned long)bufferStart;

    if (status == aditof::Status::GENERIC_ERROR) {
        return 0;
    }

    ret = ioctl(udev->uvc_fd, VIDIOC_QBUF, &ubuf);
    if (ret < 0) {
        /* Check for a USB disconnect/shutdown event. */
        if (errno == ENODEV) {
            udev->uvc_shutdown_requested = 1;
            SensorStartedStreaming = false;
            printf("UVC: Possible USB shutdown requested from "
                   "Host, seen during VIDIOC_QBUF\n");
            return 0;
        } else {
            printf("VIDIOC_QBUF on udev->uvc_fd returns: %s \n",
                   strerror(errno));
            return ret;
        }
    } else {
        // printf("VIDIOC_QBUF on udev->uvc_fd succesfull!\n");
    }

    udev->hasNewFrame = false;

    udev->qbuf_count++;

#ifdef ENABLE_BUFFER_DEBUG
    if (dev->udev->qbuf_count % LOG_FREQ == 1)
        printf("Queued buffer at UVC side = %llu\n", dev->udev->qbuf_count);
#endif

    if (!udev->first_buffer_queued && !udev->run_standalone) {
        uvc_video_stream(udev, 1);

        udev->first_buffer_queued = 1;
        udev->is_streaming = 1;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * UVC generic stuff
 */

static int uvc_video_set_format(struct uvc_device *dev) {
    struct v4l2_format fmt;
    int ret;

    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = dev->width;
    fmt.fmt.pix.height = dev->height;
    fmt.fmt.pix.pixelformat = dev->fcc;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (dev->fcc == V4L2_PIX_FMT_MJPEG)
        fmt.fmt.pix.sizeimage = dev->imgsize * 1.5;

    ret = ioctl(dev->uvc_fd, VIDIOC_S_FMT, &fmt);
    if (ret < 0) {
        printf("UVC: Unable to set format %s (%d).\n", strerror(errno), errno);
        return ret;
    }

    printf("UVC: Setting format to: %c%c%c%c %ux%u\n", pixfmtstr(dev->fcc),
           dev->width, dev->height);

    return 0;
}

static int uvc_video_stream(struct uvc_device *dev, int enable) {
    int type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    int ret;

    if (!enable) {
        ret = ioctl(dev->uvc_fd, VIDIOC_STREAMOFF, &type);
        if (ret < 0) {
            printf("UVC: VIDIOC_STREAMOFF failed: %s (%d).\n", strerror(errno),
                   errno);
            return ret;
        }

        printf("UVC: Stopping video stream.\n");

        return 0;
    }

    printf("UVC: VIDIOC_STREAMON starts.\n");

    ret = ioctl(dev->uvc_fd, VIDIOC_STREAMON, &type);
    if (ret < 0) {
        printf("UVC: Unable to start streaming %s (%d).\n", strerror(errno),
               errno);
        return ret;
    }

    printf("UVC: VIDIOC_STREAMON done.\n");

    dev->uvc_shutdown_requested = 0;

    return 0;
}

static int uvc_uninit_device(struct uvc_device *dev) {
    struct v4l2_buffer ubuf;
    unsigned int i, count;
    int ret;

    switch (dev->io) {
    case IO_METHOD_MMAP:
        for (i = 0; i < dev->nbufs; ++i) {
            ret = munmap(dev->mem[i].start, dev->mem[i].length);
            if (ret < 0) {
                printf("UVC: munmap failed\n");
                return ret;
            }
        }

        free(dev->mem);
        break;

    case IO_METHOD_USERPTR:
        if (dev->run_standalone) {
            for (i = 0; i < dev->nbufs; ++i)
                free(dev->dummy_buf[i].start);

            free(dev->dummy_buf);
        }
        /* Ensure all buffer Qed into UVC are DQed & returned to V4L2 */
        /* It appears the DQ calls may fail, just ensure the we have tried DQing
         * all buffers */
        for (count = 0; count < dev->nbufs; count++) {
            CLEAR(ubuf);
            ubuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            ubuf.memory = V4L2_MEMORY_USERPTR;

            /* Dequeue the spent buffer from UVC domain */
            ret = ioctl(dev->uvc_fd, VIDIOC_DQBUF, &ubuf);
            if (ret < 0) {
                /* No more buffers with UVC */
                printf("UVC: Unable to dequeue buffer\n");
                // return 0;
                continue;
            } else {
                printf("UVC: DQed ubuf.index: %d\n", ubuf.index);
            }
        }
        break;
    default:
        break;
    }

    return 0;
}
static int uvc_open(struct uvc_device **uvc, const char *devname) {
    struct uvc_device *dev;
    struct v4l2_capability cap;
    int fd;
    int ret = -EINVAL;

    fd = open(devname, O_RDWR | O_NONBLOCK);
    if (fd == -1) {
        printf("UVC: device open failed: %s (%d).\n", strerror(errno), errno);
        return ret;
    }

    ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
    if (ret < 0) {
        printf("UVC: unable to query uvc device: %s (%d)\n", strerror(errno),
               errno);
        goto err;
    }

    if (!(cap.capabilities &
          (V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_VIDEO_OUTPUT_MPLANE))) {
        printf("UVC: %s is no video output device\n", devname);
        goto err;
    }

    dev = static_cast<uvc_device *>(calloc(1, sizeof *dev));
    if (dev == NULL) {
        ret = -ENOMEM;
        goto err;
    }

    printf("uvc device is %s on bus %s\n", cap.card, cap.bus_info);
    printf("uvc open succeeded, file descriptor = %d\n", fd);

    dev->uvc_fd = fd;
    *uvc = dev;

    return 0;

err:
    close(fd);
    return ret;
}

static void uvc_close(struct uvc_device *dev) {
    close(dev->uvc_fd);
    free(dev->imgdata);
    free(dev);
}

/* ---------------------------------------------------------------------------
 * UVC streaming related
 */

static void uvc_video_fill_buffer(struct uvc_device *dev,
                                  struct v4l2_buffer *buf) {
    unsigned int bpl;
    unsigned int color;
    unsigned int i;

    switch (dev->fcc) {
    case V4L2_PIX_FMT_YUYV:
        /* Fill the buffer with video data. */
        bpl = dev->width * 2;
        color = dev->color;
        for (i = 0; i < dev->height; i++) {
            /*TODO: Improve this logic to show more sensible dummy image */
            memset(dev->mem[buf->index].start + i * bpl, color, bpl);
            if (i % 20 == 0)
                color += 20;
        }

        buf->bytesused = bpl * dev->height;
        break;

    case V4L2_PIX_FMT_MJPEG:
        memcpy(dev->mem[buf->index].start, dev->imgdata, dev->imgsize);
        buf->bytesused = dev->imgsize;
        break;
    }
}

static int uvc_video_process(struct uvc_device *dev) {
    struct v4l2_buffer ubuf;
    struct v4l2_buffer vbuf;

    int ret;
    /*
     * Return immediately if UVC video output device has not started
     * streaming yet.
     */
    if (!dev->is_streaming)
        return 0;

    /* Prepare a v4l2 buffer to be dequeued from UVC domain. */
    CLEAR(ubuf);

    ubuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    switch (dev->io) {
    case IO_METHOD_MMAP:
        ubuf.memory = V4L2_MEMORY_MMAP;
        break;

    case IO_METHOD_USERPTR:
    default:
        ubuf.memory = V4L2_MEMORY_USERPTR;
        break;
    }

    /* UVC - V4L2 integrated path. */

    /*
     * Return immediately if V4L2 video capture device has not
     * started streaming yet or if QBUF was not called even once on
     * the UVC side.
     */
    if (!SensorStartedStreaming || !dev->first_buffer_queued) {
        // printf("Streaming is OFF\n");
        return 0;
    }

    if (!dev->uvc_shutdown_requested) {
        if (dev->hasNewFrame) {
            printf("No new frame!");
            return 0;
        }
    }

    /* Dequeue the spent buffer from UVC domain */
    ret = ioctl(dev->uvc_fd, VIDIOC_DQBUF, &ubuf);
    if (ret < 0) {
        SensorStartedStreaming = false;
        printf("UVC: Unable to dequeue buffer: %s (%d).\n", strerror(errno),
               errno);
        return ret;
    }

#ifdef ENABLE_BUFFER_DEBUG
    if (dev->dqbuf_count % LOG_FREQ == 1)
        printf("DeQueued buffer at UVC side = %llu\n", dev->dqbuf_count);
#endif

    /*
     * If the dequeued buffer was marked with state ERROR by the
     * underlying UVC driver gadget, do not queue the same to V4l2
     * and wait for a STREAMOFF event on UVC side corresponding to
     * set_alt(0). So, now all buffers pending at UVC end will be
     * dequeued one-by-one and we will enter a state where we once
     * again wait for a set_alt(1) command from the USB host side.
     */
    if (ubuf.flags & V4L2_BUF_FLAG_ERROR) {
        dev->uvc_shutdown_requested = 1;
        printf("UVC: Possible USB shutdown requested from "
               "Host, seen during VIDIOC_DQBUF\n");

        return 0;
    }

    CLEAR(vbuf);

    vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    vbuf.memory = V4L2_MEMORY_MMAP;
    vbuf.index = ubuf.index;
    vbuf.m.planes = gPlanes;
    vbuf.length = 1;

    sensorV4lBufAccess->enqueueInternalBuffer(vbuf);

    dev->hasNewFrame = true;

#ifdef ENABLE_BUFFER_DEBUG
    if (dev->vdev->qbuf_count % LOG_FREQ == 1)
        printf("ReQueueing buffer at V4L2 side = %llu\n",
               dev->vdev->qbuf_count);
#endif

    return 0;
}

static int uvc_video_qbuf_mmap(struct uvc_device *dev) {
    unsigned int i;
    int ret;

    for (i = 0; i < dev->nbufs; ++i) {
        memset(&dev->mem[i].buf, 0, sizeof(dev->mem[i].buf));

        dev->mem[i].buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        dev->mem[i].buf.memory = V4L2_MEMORY_MMAP;
        dev->mem[i].buf.index = i;

        /* UVC standalone setup. */
        if (dev->run_standalone)
            uvc_video_fill_buffer(dev, &(dev->mem[i].buf));

        ret = ioctl(dev->uvc_fd, VIDIOC_QBUF, &(dev->mem[i].buf));
        if (ret < 0) {
            printf("UVC: VIDIOC_QBUF failed : %s (%d).\n", strerror(errno),
                   errno);
            return ret;
        }

        dev->qbuf_count++;
    }

    return 0;
}

static int uvc_video_qbuf_userptr(struct uvc_device *dev) {
    unsigned int i;
    int ret;
    printf("Entered qbuf userptr\n");
    /* UVC standalone setup. */
    if (dev->run_standalone) {
        for (i = 0; i < dev->nbufs; ++i) {
            struct v4l2_buffer buf;

            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
            buf.memory = V4L2_MEMORY_USERPTR;
            buf.m.userptr = (unsigned long)dev->dummy_buf[i].start;
            buf.length = dev->dummy_buf[i].length;
            buf.index = i;

            ret = ioctl(dev->uvc_fd, VIDIOC_QBUF, &buf);
            if (ret < 0) {
                printf("UVC: VIDIOC_QBUF failed : %s (%d).\n", strerror(errno),
                       errno);
                return ret;
            }

            dev->qbuf_count++;
        }
    }

    printf("Exited qbuf userptr\n");
    return 0;
}

static int uvc_video_qbuf(struct uvc_device *dev) {
    int ret = 0;

    switch (dev->io) {
    case IO_METHOD_MMAP:
        printf("IO_METHOD_MMAP will call uvc_video_qbuf_mmap!");
        ret = uvc_video_qbuf_mmap(dev);
        break;

    case IO_METHOD_USERPTR:
        printf("IO_METHOD_USERPTR will call uvc_video_qbuf_userptr!");
        ret = uvc_video_qbuf_userptr(dev);
        break;

    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

static int uvc_video_reqbufs_mmap(struct uvc_device *dev, int nbufs) {
    struct v4l2_requestbuffers rb;
    unsigned int i;
    int ret;

    CLEAR(rb);

    rb.count = nbufs;
    rb.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    rb.memory = V4L2_MEMORY_MMAP;

    ret = ioctl(dev->uvc_fd, VIDIOC_REQBUFS, &rb);
    if (ret < 0) {
        if (ret == -EINVAL)
            printf("UVC: does not support memory mapping\n");
        else
            printf("UVC: Unable to allocate buffers: %s (%d).\n",
                   strerror(errno), errno);
        goto err;
    }

    if (!rb.count)
        return 0;

    if (rb.count < 2) {
        printf("UVC: Insufficient buffer memory.\n");
        ret = -EINVAL;
        goto err;
    }

    /* Map the buffers. */
    dev->mem = static_cast<buffer *>(calloc(rb.count, sizeof dev->mem[0]));
    if (!dev->mem) {
        printf("UVC: Out of memory\n");
        ret = -ENOMEM;
        goto err;
    }

    for (i = 0; i < rb.count; ++i) {
        memset(&dev->mem[i].buf, 0, sizeof(dev->mem[i].buf));

        dev->mem[i].buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        dev->mem[i].buf.memory = V4L2_MEMORY_MMAP;
        dev->mem[i].buf.index = i;

        ret = ioctl(dev->uvc_fd, VIDIOC_QUERYBUF, &(dev->mem[i].buf));
        if (ret < 0) {
            printf("UVC: VIDIOC_QUERYBUF failed for buf %d: "
                   "%s (%d).\n",
                   i, strerror(errno), errno);
            ret = -EINVAL;
            goto err_free;
        }
        dev->mem[i].start = mmap(
            NULL /* start anywhere */, dev->mem[i].buf.length,
            PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */,
            dev->uvc_fd, dev->mem[i].buf.m.offset);

        if (MAP_FAILED == dev->mem[i].start) {
            printf("UVC: Unable to map buffer %u: %s (%d).\n", i,
                   strerror(errno), errno);
            dev->mem[i].length = 0;
            ret = -EINVAL;
            goto err_free;
        }

        dev->mem[i].length = dev->mem[i].buf.length;
        printf("UVC: Buffer %u mapped at address %p, size: %lu.\n", i,
               dev->mem[i].start, dev->mem[i].length);
    }

    dev->nbufs = rb.count;

    return 0;

err_free:
    free(dev->mem);
err:
    return ret;
}

static int uvc_video_reqbufs_userptr(struct uvc_device *dev, int nbufs) {
    struct v4l2_requestbuffers rb;
    int ret;

    CLEAR(rb);

    rb.count = nbufs;
    rb.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    rb.memory = V4L2_MEMORY_USERPTR;

    ret = ioctl(dev->uvc_fd, VIDIOC_REQBUFS, &rb);
    if (ret < 0) {
        if (ret == -EINVAL)
            printf("UVC: does not support user pointer i/o\n");
        else
            printf("UVC: VIDIOC_REQBUFS error %s (%d).\n", strerror(errno),
                   errno);
        goto err;
    }

    if (!rb.count)
        return 0;

    dev->nbufs = rb.count;

    return 0;

err:
    return ret;
}

static int uvc_video_reqbufs(struct uvc_device *dev, int nbufs) {
    int ret = 0;

    switch (dev->io) {
    case IO_METHOD_MMAP:
        ret = uvc_video_reqbufs_mmap(dev, nbufs);
        break;

    case IO_METHOD_USERPTR:
        ret = uvc_video_reqbufs_userptr(dev, nbufs);
        break;

    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

/*
 * This function is called in response to either:
 * 	- A SET_ALT(interface 1, alt setting 1) command from USB host,
 * 	  if the UVC gadget supports an ISOCHRONOUS video streaming endpoint
 * 	  or,
 *
 *	- A UVC_VS_COMMIT_CONTROL command from USB host, if the UVC gadget
 *	  supports a BULK type video streaming endpoint.
 */
static int uvc_handle_streamon_event(struct uvc_device *dev) {
    int ret;

    ret = uvc_video_reqbufs(dev, dev->nbufs);
    if (ret < 0)
        goto err;

    if (!dev->run_standalone) {
        camDepthSensor->start();

        SensorStartedStreaming = true;

        dev->hasNewFrame = true;
    }

    /* Queue buffers to UVC domain and start streaming. */
    ret = uvc_video_qbuf(dev);
    if (ret < 0)
        goto err;

    if (dev->run_standalone) {
        uvc_video_stream(dev, 1);
        dev->first_buffer_queued = 1;
        dev->is_streaming = 1;
    }

    return 0;

err:
    return ret;
}

/* ---------------------------------------------------------------------------
 * UVC Request processing
 */

static void uvc_fill_streaming_control(struct uvc_device *dev,
                                       struct uvc_streaming_control *ctrl,
                                       int iframe, int iformat) {
    const struct uvc_format_info *format;
    const struct uvc_frame_info *frame;
    unsigned int nframes;

    if (iformat < 0)
        iformat = ARRAY_SIZE(uvc_formats) + iformat;
    if (iformat < 0 || iformat >= (int)ARRAY_SIZE(uvc_formats))
        return;
    format = &uvc_formats[iformat];

    nframes = 0;
    while (format->frames[nframes].width != 0)
        ++nframes;

    if (iframe < 0)
        iframe = nframes + iframe;
    if (iframe < 0 || iframe >= (int)nframes)
        return;
    frame = &format->frames[iframe];

    memset(ctrl, 0, sizeof *ctrl);

    ctrl->bmHint = 1;
    ctrl->bFormatIndex = iformat + 1;
    ctrl->bFrameIndex = iframe + 1;
    ctrl->dwFrameInterval = frame->intervals[0];
    switch (format->fcc) {
    case V4L2_PIX_FMT_YUYV:
        ctrl->dwMaxVideoFrameSize = frame->width * frame->height * 2;
        break;
    case V4L2_PIX_FMT_MJPEG:
        ctrl->dwMaxVideoFrameSize = dev->imgsize;
        break;
    }

    /* TODO: the UVC maxpayload transfer size should be filled
     * by the driver.
     */
    if (!dev->bulk)
        ctrl->dwMaxPayloadTransferSize =
            (dev->maxpkt) * (dev->mult + 1) * (dev->burst + 1);
    else
        ctrl->dwMaxPayloadTransferSize = ctrl->dwMaxVideoFrameSize;

    ctrl->bmFramingInfo = 3;
    ctrl->bPreferedVersion = 1;
    ctrl->bMaxVersion = 1;
}

static void uvc_events_process_standard(struct uvc_device *dev,
                                        struct usb_ctrlrequest *ctrl,
                                        struct uvc_request_data *resp) {
    printf("standard request\n");
    (void)dev;
    (void)ctrl;
    (void)resp;
}

static void uvc_events_process_control(struct uvc_device *dev, uint8_t req,
                                       uint8_t cs, uint8_t entity_id,
                                       uint8_t len,
                                       struct uvc_request_data *resp) {
    switch (entity_id) {
    case 0:
        switch (cs) {
        case UVC_VC_REQUEST_ERROR_CODE_CONTROL:
            /* Send the request error code last prepared. */
            resp->data[0] = dev->request_error_code.data[0];
            resp->length = dev->request_error_code.length;
            break;

        default:
            /*
             * If we were not supposed to handle this
             * 'cs', prepare an error code response.
             */
            SET_REQ_ERROR_CODE(0x06, 1);
            break;
        }
        break;

    /* Camera terminal unit 'UVC_VC_INPUT_TERMINAL'. */
    case 1:
        SET_REQ_ERROR_CODE(0x00, 1);
        switch (cs) {
        /*
         * We support only 'UVC_CT_AE_MODE_CONTROL' for CAMERA
         * terminal, as our bmControls[0] = 2 for CT. Also we
         * support only auto exposure.
         */
        case UVC_CT_AE_MODE_CONTROL:
            switch (req) {
            case UVC_SET_CUR:
                USB_REQ_DEBUG("Received SET_CUR %d\n", cs);

                /* Incase of auto exposure, attempts to
                 * programmatically set the auto-adjusted
                 * controls are ignored.
                 */
                resp->data[0] = 0x01;
                resp->length = 1;
                break;

            case UVC_GET_INFO:
                /*
                 * TODO: We support Set and Get requests, but
                 * don't support async updates on an video
                 * status (interrupt) endpoint as of
                 * now.
                 */
                USB_REQ_DEBUG("Received GET_INFO %d\n", cs);

                resp->data[0] = 0x03;
                resp->length = 1;
                break;

            case UVC_GET_CUR:
            case UVC_GET_DEF:
            case UVC_GET_RES:
                USB_REQ_DEBUG("Received %x on %d\n", req, cs);

                /* Auto Mode â€“ auto Exposure Time, auto Iris. */
                resp->data[0] = 0x02;
                resp->length = 1;
                break;
            default:
                USB_REQ_DEBUG("Unsupported: Received %x on %d\n", req, cs);

                /*
                 * We don't support this control, so STALL the
                 * control ep.
                 */
                resp->length = -EL2HLT;
                /*
                 * For every unsupported control request
                 * set the request error code to appropriate
                 * value.
                 */
                SET_REQ_ERROR_CODE(0x07, 1);
                break;
            }
            break;

        default:
            /*
             * We don't support this control, so STALL the control
             * ep.
             */
            resp->length = -EL2HLT;
            /*
             * If we were not supposed to handle this
             * 'cs', prepare a Request Error Code response.
             */
            SET_REQ_ERROR_CODE(0x06, 1);
            break;
        }
        break;

    /* processing unit 'UVC_VC_PROCESSING_UNIT' */
    case 2:
        SET_REQ_ERROR_CODE(0x00, 1);
        switch (cs) {
        /*
         * We support only 'UVC_PU_BRIGHTNESS_CONTROL' for Processing
         * Unit, as our bmControls[0] = 1 for PU.
         */
        case UVC_PU_BRIGHTNESS_CONTROL:
            switch (req) {
            case UVC_SET_CUR:
                resp->data[0] = 0x0;
                resp->length = len;
                break;
            case UVC_GET_MIN:
                resp->data[0] = PU_BRIGHTNESS_MIN_VAL;
                resp->length = 2;
                break;
            case UVC_GET_MAX:
                resp->data[0] = PU_BRIGHTNESS_MAX_VAL;
                resp->length = 2;
                break;
            case UVC_GET_CUR:
                resp->length = 2;
                memcpy(&resp->data[0], &dev->brightness_val, resp->length);
                break;
            case UVC_GET_INFO:
                /*
                 * We support Set and Get requests and don't
                 * support async updates on an interrupt endpt
                 */
                resp->data[0] = 0x03;
                resp->length = 1;
                break;
            case UVC_GET_DEF:
                resp->data[0] = PU_BRIGHTNESS_DEFAULT_VAL;
                resp->length = 2;
                break;
            case UVC_GET_RES:
                resp->data[0] = PU_BRIGHTNESS_STEP_SIZE;
                resp->length = 2;
                break;
            default:
                /*
                 * We don't support this control, so STALL the
                 * default control ep.
                 */
                resp->length = -EL2HLT;
                /*
                 * For every unsupported control request
                 * set the request error code to appropriate
                 * code.
                 */
                SET_REQ_ERROR_CODE(0x07, 1);
                break;
            }
            break;

        default:
            /*
             * We don't support this control, so STALL the control
             * ep.
             */
            resp->length = -EL2HLT;
            /*
             * If we were not supposed to handle this
             * 'cs', prepare a Request Error Code response.
             */
            SET_REQ_ERROR_CODE(0x06, 1);
            break;
        }

        break;

    /* extension unit 'UVC_VC_EXTENSION_UNIT' */
    case 3:
        SET_REQ_ERROR_CODE(0x00, 1);
        switch (cs) {
        case 1: /* AFE Programming */
        case 2: /* AFE Register Read */
            switch (req) {
            case UVC_SET_CUR:
                USB_REQ_DEBUG("Received SET_CUR on %d\n", cs);

                dev->set_cur_cs = cs;
                resp->data[0] = 0x0;
                resp->length = len;
                break;
            case UVC_GET_CUR:
                USB_REQ_DEBUG("Received GET_CUR on %d\n", cs);

                if (cs == 2) {
                    camDepthSensor->readAfeRegisters(
                        &reg_addr,
                        reinterpret_cast<unsigned short *>(resp->data), 1);
                    resp->length = 2;
                } else {
                    resp->length = 2;
                    memcpy(&resp->data[0], &dev->brightness_val, resp->length);
                }
                break;

            case UVC_GET_INFO:
                USB_REQ_DEBUG("Received GET_INFO on %d\n", cs);

                /*
                 * We support Set and Get requests and don't
                 * support async updates on an interrupt endpt
                 */
                resp->data[0] = 0x03;
                resp->length = 1;
                break;
            case UVC_GET_LEN:
                USB_REQ_DEBUG("Received GET_LEN on %d\n", cs);

                if (cs == 2) {
                    resp->data[0] = 0x02; // 2 bytes
                    resp->data[1] = 0x00;

                    USB_REQ_DEBUG("Responding with size %x for cs: %d\n",
                                  resp->data[0], cs);
                } else {
                    resp->data[0] = MAX_PACKET_SIZE; // 60 bytes
                    resp->data[1] = 0x00;

                    USB_REQ_DEBUG("Responding with size %x for cs: %d\n",
                                  resp->data[0], cs);
                }
                resp->length = 2;
                break;

            case UVC_GET_MIN:
            case UVC_GET_MAX:
            case UVC_GET_DEF:
            case UVC_GET_RES:
                USB_REQ_DEBUG("Received %x on %d\n", req, cs);

                resp->data[0] = 0xff;
                resp->length = 1;
                break;

            default:
                printf("Unsupported bRequest: Received bRequest %x on cs %d\n",
                       req, cs);
                /*
                 * We don't support this control, so STALL the
                 * default control ep.
                 */
                resp->length = -EL2HLT;
                /*
                 * For every unsupported control request
                 * set the request error code to appropriate
                 * code.
                 */
                SET_REQ_ERROR_CODE(0x07, 1);
                break;
            }
            break;

        case 3: /* TempSensor Read */
            switch (req) {
            case UVC_SET_CUR:
                USB_REQ_DEBUG("Received SET_CUR on %d\n", cs);
                dev->set_cur_cs = cs;
                resp->data[0] = 0x0;
                resp->length = len;
                break;

            case UVC_GET_CUR:
                USB_REQ_DEBUG("Received GET_CUR on %d\n", cs);

                float temperature;
                temperatureSensors[temp_index]->read(temperature);
                *((float *)resp->data) = temperature;
                resp->length = 4;
                break;

            case UVC_GET_INFO:
                USB_REQ_DEBUG("Received GET_INFO on %d\n", cs);

                /*
                 * We support Set and Get requests and don't
                 * support async updates on an interrupt endpt
                 */
                resp->data[0] = 0x03;
                resp->length = 1;
                break;

            case UVC_GET_LEN:
                USB_REQ_DEBUG("Received GET_LEN on %d\n", cs);

                resp->data[0] = 0x04; // 4 bytes
                resp->data[1] = 0x00;
                resp->length = 2;

                USB_REQ_DEBUG("Responding with size %x for cs: %d\n",
                              resp->data[0], cs);
                break;

            case UVC_GET_MIN:
            case UVC_GET_MAX:
            case UVC_GET_DEF:
            case UVC_GET_RES:
                USB_REQ_DEBUG("Received %x on %d\n", req, cs);

                resp->data[0] = 0xff;
                resp->length = 1;
                break;

            default:
                printf("Unsupported bRequest: Received bRequest %x on cs %d\n",
                       req, cs);
                /*
                 * We don't support this control, so STALL the
                 * default control ep.
                 */
                resp->length = -EL2HLT;
                /*
                 * For every unsupported control request
                 * set the request error code to appropriate
                 * code.
                 */
                SET_REQ_ERROR_CODE(0x07, 1);
                break;
            }
            break;

        case 4: /* Get advertised sensors */
            switch (req) {
            case UVC_SET_CUR:
                dev->set_cur_cs = cs;
                resp->data[0] = 0x0;
                resp->length = len;
                break;

            case UVC_GET_CUR:
                memcpy(resp->data, sensorsInfoBuffer + sensors_read_pos,
                       sensors_read_len);
                resp->length = sensors_read_len;
                break;

            case UVC_GET_INFO:
                /*
                 * We support Set and Get requests and don't
                 * support async updates on an interrupt endpt
                 */
                resp->data[0] = 0x03;
                resp->length = 1;
                break;

            case UVC_GET_LEN:
                resp->data[0] = MAX_PACKET_SIZE;
                resp->data[1] = 0x0;
                resp->length = 2;
                break;
            case UVC_GET_MIN:
            case UVC_GET_MAX:
            case UVC_GET_DEF:
            case UVC_GET_RES:
                USB_REQ_DEBUG("Received %x on %d\n", req, cs);

                resp->data[0] = 0xff;
                resp->length = 1;
                break;
            default:
                printf("Unsupported bRequest: Received bRequest %x on cs %d\n",
                       req, cs);
                /*
                 * We don't support this control, so STALL the
                 * default control ep.
                 */
                resp->length = -EL2HLT;
                /*
                 * For every unsupported control request
                 * set the request error code to appropriate
                 * code.
                 */
                SET_REQ_ERROR_CODE(0x07, 1);
                break;
            } //switch (req)
            break;

        case 5: /* EEPROM Read */
        case 6: /* EEPROM Write */
            switch (req) {
            case UVC_SET_CUR:
                USB_REQ_DEBUG("Received SET_CUR on %d\n", cs);
                dev->set_cur_cs = cs;

                resp->data[0] = 0x0;
                resp->length = len;
                break;

            case UVC_GET_CUR:
                USB_REQ_DEBUG("Received GET_CUR on %d\n", cs);

                if (cs == 5) {
                    storages[storage_index]->read(eeprom_read_addr, resp->data,
                                                  eeprom_read_len);
                    resp->length = eeprom_read_len;
                }
                break;

            case UVC_GET_INFO:
                USB_REQ_DEBUG("Received GET_INFO on %d\n", cs);

                /*
                 * We support Set and Get requests and don't
                 * support async updates on an interrupt endpt
                 */
                resp->data[0] = 0x03;
                resp->length = 1;
                break;

            case UVC_GET_LEN:
                USB_REQ_DEBUG("Received GET_LEN on %d\n", cs);

                if (cs == 6 || cs == 5) {
                    resp->data[0] = MAX_PACKET_SIZE; // 2 bytes
                } else {
                    resp->data[0] = 0x02; // 2 bytes
                }
                resp->data[1] = 0x00;
                resp->length = 2;

                USB_REQ_DEBUG("Responding with size %x for cs: %d\n",
                              resp->data[0], cs);
                break;

            case UVC_GET_MIN:
            case UVC_GET_MAX:
            case UVC_GET_DEF:
            case UVC_GET_RES:
                USB_REQ_DEBUG("Received %x on %d\n", req, cs);

                resp->data[0] = 0xff;
                resp->length = 1;
                break;

            default:
                printf("Unsupported bRequest: Received bRequest %x on cs %d\n",
                       req, cs);
                /*
                 * We don't support this control, so STALL the
                 * default control ep.
                 */
                resp->length = -EL2HLT;
                /*
                 * For every unsupported control request
                 * set the request error code to appropriate
                 * code.
                 */
                SET_REQ_ERROR_CODE(0x07, 1);
                break;
            }
            break;

        case 7: /* Ready for writing to EEPROM */
            switch (req) {
            case UVC_SET_CUR:
                resp->data[0] = 0x0;
                resp->length = len;
                break;

            case UVC_GET_CUR:
                resp->data[0] = eeprom_write_ready.load() ? 1 : 0;
                resp->length = 1;
                break;

            case UVC_GET_INFO:
                /*
                 * We support Set and Get requests and don't
                 * support async updates on an interrupt endpt
                 */
                resp->data[0] = 0x03;
                resp->length = 1;
                break;

            case UVC_GET_LEN:
                resp->data[0] = 0x01; // 1 byte
                resp->data[1] = 0x00;
                resp->length = 2;
                break;

            case UVC_GET_MIN:
            case UVC_GET_MAX:
            case UVC_GET_DEF:
            case UVC_GET_RES:
                resp->data[0] = 0xff;
                resp->length = 1;
                break;

            default:
                printf("Unsupported bRequest: Received bRequest %x on cs %d\n",
                       req, cs);
                /*
                 * We don't support this control, so STALL the
                 * default control ep.
                 */
                resp->length = -EL2HLT;
                /*
                 * For every unsupported control request
                 * set the request error code to appropriate
                 * code.
                 */
                SET_REQ_ERROR_CODE(0x07, 1);
                break;
            }
            break;

        default:
            printf("Unsupported cs: Received cs %d \n", cs);
            /*
             * We don't support this control, so STALL the control
             * ep.
             */
            resp->length = -EL2HLT;
            /*
             * If we were not supposed to handle this
             * 'cs', prepare a Request Error Code response.
             */
            SET_REQ_ERROR_CODE(0x06, 1);
            break;
        }

        break;

    default:
        printf("Unsupported entity: Reeived control on invalid entity id %d\n",
               entity_id);
        /*
         * If we were not supposed to handle this
         * 'cs', prepare a Request Error Code response.
         */
        SET_REQ_ERROR_CODE(0x06, 1);
        break;
    }

    // printf("control request (req %02x cs %02x)\n", req, cs);
}

static void uvc_events_process_streaming(struct uvc_device *dev, uint8_t req,
                                         uint8_t cs,
                                         struct uvc_request_data *resp) {
    struct uvc_streaming_control *ctrl;

    printf("streaming request (req %02x cs %02x)\n", req, cs);

    if (cs != UVC_VS_PROBE_CONTROL && cs != UVC_VS_COMMIT_CONTROL)
        return;

    ctrl = (struct uvc_streaming_control *)&resp->data;
    resp->length = sizeof *ctrl;

    switch (req) {
    case UVC_SET_CUR:
        dev->control = cs;
        resp->length = 34;
        break;

    case UVC_GET_CUR:
        if (cs == UVC_VS_PROBE_CONTROL)
            memcpy(ctrl, &dev->probe, sizeof *ctrl);
        else
            memcpy(ctrl, &dev->commit, sizeof *ctrl);
        break;

    case UVC_GET_MIN:
    case UVC_GET_MAX:
    case UVC_GET_DEF:
        uvc_fill_streaming_control(dev, ctrl, req == UVC_GET_MAX ? -1 : 0,
                                   req == UVC_GET_MAX ? -1 : 0);
        break;

    case UVC_GET_RES:
        CLEAR(ctrl);
        break;

    case UVC_GET_LEN:
        resp->data[0] = 0x00;
        resp->data[1] = 0x22;
        resp->length = 2;
        break;

    case UVC_GET_INFO:
        resp->data[0] = 0x03;
        resp->length = 1;
        break;
    }
}

static void uvc_events_process_class(struct uvc_device *dev,
                                     struct usb_ctrlrequest *ctrl,
                                     struct uvc_request_data *resp) {
    if ((ctrl->bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE)
        return;

    switch (ctrl->wIndex & 0xff) {
    case UVC_INTF_CONTROL:
        // printf("events_process_controll\n");
        uvc_events_process_control(dev, ctrl->bRequest, ctrl->wValue >> 8,
                                   ctrl->wIndex >> 8, ctrl->wLength, resp);
        break;

    case UVC_INTF_STREAMING:
        // printf("events_process_streaming\n");
        uvc_events_process_streaming(dev, ctrl->bRequest, ctrl->wValue >> 8,
                                     resp);
        break;

    default:
        break;
    }
}
static void uvc_events_process_setup(struct uvc_device *dev,
                                     struct usb_ctrlrequest *ctrl,
                                     struct uvc_request_data *resp) {
    dev->control = 0;

    USB_REQ_DEBUG("\nbRequestType %02x bRequest %02x wValue %04x wIndex %04x "
                  "wLength %04x\n",
                  ctrl->bRequestType, ctrl->bRequest, ctrl->wValue,
                  ctrl->wIndex, ctrl->wLength);

    switch (ctrl->bRequestType & USB_TYPE_MASK) {
    case USB_TYPE_STANDARD:
        // printf("process standard!\n");
        uvc_events_process_standard(dev, ctrl, resp);
        break;

    case USB_TYPE_CLASS:
        // printf("process class!\n");
        uvc_events_process_class(dev, ctrl, resp);
        break;

    default:
        break;
    }
}

//static int uvc_events_process_control_data(struct uvc_device *dev, uint8_t cs,
//                                           uint8_t entity_id,
//                                           struct uvc_request_data *data) {
//    switch (entity_id) {
//    /* Processing unit 'UVC_VC_PROCESSING_UNIT'. */
//    case 2:
//        switch (cs) {
//        /*
//         * We support only 'UVC_PU_BRIGHTNESS_CONTROL' for Processing
//         * Unit, as our bmControls[0] = 1 for PU.
//         */
//        case UVC_PU_BRIGHTNESS_CONTROL:
//            memcpy(&dev->brightness_val, data->data, data->length);
//            /* UVC - V4L2 integrated path. */
//            if (!dev->run_standalone)
//                /*
//                 * Try to change the Brightness attribute on
//                 * Video capture device. Note that this try may
//                 * succeed or end up with some error on the
//                 * video capture side. By default to keep tools
//                 * like USBCV's UVC test suite happy, we are
//                 * maintaining a local copy of the current
//                 * brightness value in 'dev->brightness_val'
//                 * variable and we return the same value to the
//                 * Host on receiving a GET_CUR(BRIGHTNESS)
//                 * control request.
//                 *
//                 * FIXME: Keeping in view the point discussed
//                 * above, notice that we ignore the return value
//                 * from the function call below. To be strictly
//                 * compliant, we should return the same value
//                 * accordingly.
//                 */
//                // v4l2_set_ctrl(dev->vdev, dev->brightness_val,
//                //   V4L2_CID_BRIGHTNESS);
//                break;

//        default:
//            break;
//        }

//        break;

//    default:
//        break;
//    }

//    printf("Control Request data phase (cs %02x entity %02x)\n", cs, entity_id);

//    return 0;
//}

static int uvc_events_process_data(struct uvc_device *dev,
                                   struct uvc_request_data *data) {
    struct uvc_streaming_control *target;
    struct uvc_streaming_control *ctrl;
    const struct uvc_format_info *format;
    const struct uvc_frame_info *frame;
    const unsigned int *interval;
    unsigned int iformat, iframe;
    unsigned int nframes;

    switch (dev->control) {
    case UVC_VS_PROBE_CONTROL:
        printf("setting probe control, length = %d\n", data->length);
        target = &dev->probe;
        break;

    case UVC_VS_COMMIT_CONTROL:
        printf("setting commit control, length = %d\n", data->length);
        target = &dev->commit;
        break;

    default:
        USB_REQ_DEBUG("control: %d, length: %d cs: %d\n", dev->control,
                      data->length, dev->set_cur_cs);

        if (ad903x_hw) {
            if (dev->set_cur_cs == 1) { /* Firmware data */
                /* Store the firmware data from Host and flash it when enough is
                 * available */

                static bool shouldRestartSensor = false;

                if (SensorStartedStreaming) {
                    shouldRestartSensor = true;
                    SensorStartedStreaming = false;
                }

                firmware_size = data->data[1];
                memcpy(&firmware[flen], &data->data[2], firmware_size);
                flen += firmware_size;

                if (data->data[0] == 0x02) {
                    camDepthSensor->start(); // make sure the device is started
                    aditof::Status status = camDepthSensor->program(
                        reinterpret_cast<uint8_t *>(firmware), flen);
                    if (status == aditof::Status::OK) {
                        printf("Flashed firmware, flen %d\n", flen);
                    } else {
                        printf("Error Programing the chip\n");
                    }
                    if (shouldRestartSensor) {
                        SensorStartedStreaming = true;
                        shouldRestartSensor = false;
                    }
                    flen = 0;
                    memset(&firmware[flen], 0, FIRMWARE_CAPACITY);
                }
            } else if (dev->set_cur_cs == 2) { /* AFE register address */
                reg_addr = *((unsigned short *)(data->data));
            } else if (dev->set_cur_cs == 3) { /* Temperature index */
                temp_index = data->data[0];
            } else if (dev->set_cur_cs == 4) {
                sensors_read_pos = *((unsigned int *)(data->data));
                sensors_read_len = data->data[4];
            } else if (dev->set_cur_cs == 5) { /* Storage index, read address */
                storage_index = data->data[0];
                eeprom_read_addr = *((unsigned int *)&(data->data[1]));
                eeprom_read_len = data->data[5];
            } else if (dev->set_cur_cs == 6) {
                if (eeprom_write_ready.load()) {
                    if (eeprom_write_len == 0) {
                        storage_index = data->data[0];
                        eeprom_write_addr = *((unsigned int *)&(data->data[1]));
                    }
                    memcpy(&eeprom_data[eeprom_write_len], &data->data[6],
                           data->data[5]);
                    eeprom_write_len += data->data[5];
                    if (data->data[5] < MAX_PACKET_SIZE - 6) {
                        if (eepromWriteThread.joinable()) {
                            eepromWriteThread.join();
                        }
                        eepromWriteThread = std::thread(
                            [=](std::shared_ptr<aditof::StorageInterface> e) {
                                eeprom_write_ready = false;
                                e->write(eeprom_write_addr, eeprom_data,
                                         eeprom_write_len);
                                eeprom_write_ready = true;
                                eeprom_write_len = 0;
                            },
                            storages[storage_index]);
                    }
                }
            }
            dev->set_cur_cs = 0;
            return 0;
        } else {
            /*
             * As we support only BRIGHTNESS control, this request is
             * for setting BRIGHTNESS control.
             * Check for any invalid SET_CUR(BRIGHTNESS) requests
             * from Host. Note that we support Brightness levels
             * from 0x0 to 0x10 in a step of 0x1. So, any request
             * with value greater than 0x10 is invalid.
             */
            return -EINVAL;
#if 0
          if (*(unsigned int *)data->data > PU_BRIGHTNESS_MAX_VAL) {
              printf ("Returning EINVAL for Brightness control\n");
              return -EINVAL;
          } else {
              ret = uvc_events_process_control_data(dev, UVC_PU_BRIGHTNESS_CONTROL, 2, data);

              if (ret < 0)
              {
                  printf ("Returning err from uvc_events_process_control_data\n");
                  goto err;
              }

              return 0;
          }
#endif
        }
    }
    ctrl = (struct uvc_streaming_control *)&data->data;
    iformat = clamp((unsigned int)ctrl->bFormatIndex, 1U,
                    (unsigned int)ARRAY_SIZE(uvc_formats));
    format = &uvc_formats[iformat - 1];

    nframes = 0;
    while (format->frames[nframes].width != 0)
        ++nframes;

    iframe = clamp((unsigned int)ctrl->bFrameIndex, 1U, nframes);
    frame = &format->frames[iframe - 1];
    interval = frame->intervals;

    while (interval[0] < ctrl->dwFrameInterval && interval[1])
        ++interval;

    target->bFormatIndex = iformat;
    target->bFrameIndex = iframe;
    switch (format->fcc) {
    case V4L2_PIX_FMT_YUYV:
        target->dwMaxVideoFrameSize = frame->width * frame->height * 2;
        break;
    case V4L2_PIX_FMT_MJPEG:
        if (dev->imgsize == 0)
            printf("WARNING: MJPEG requested and no image loaded.\n");
        target->dwMaxVideoFrameSize = dev->imgsize;
        break;
    }
    target->dwFrameInterval = *interval;

    if (dev->control == UVC_VS_COMMIT_CONTROL) {
        dev->fcc = format->fcc;
        /* TODO: Check why commit control triggers changes to device side
         * width and height ! For now, ignore what Host wants, Device side
         * knows better */
        // dev->width = frame->width;
        // dev->height = frame->height;
    }

    return 0;
}

static void uvc_events_process(struct uvc_device *dev) {
    struct v4l2_event v4l2_event;
    struct uvc_event *uvc_event =
        reinterpret_cast<struct uvc_event *>(&v4l2_event.u.data);
    struct uvc_request_data resp;
    int ret;

    // printf("UVC process event!\n");
    ret = ioctl(dev->uvc_fd, VIDIOC_DQEVENT, &v4l2_event);
    if (ret < 0) {
        printf("VIDIOC_DQEVENT failed: %s (%d)\n", strerror(errno), errno);
        return;
    }

    memset(&resp, 0, sizeof resp);
    resp.length = -EL2HLT;

    switch (v4l2_event.type) {
    case UVC_EVENT_CONNECT:
        printf("UVC_EVENT_CONNECT received\n");
        return;

    case UVC_EVENT_DISCONNECT:
        dev->uvc_shutdown_requested = 1;
        printf("UVC: Possible USB shutdown requested from "
               "Host, seen via UVC_EVENT_DISCONNECT\n");
        return;

    case UVC_EVENT_SETUP:
        // printf("UVC_EVENT_SETUP received\n");
        uvc_events_process_setup(dev, &uvc_event->req, &resp);
        break;

    case UVC_EVENT_DATA:
        // printf("UVC_EVENT_DATA received\n");
        ret = uvc_events_process_data(dev, &uvc_event->data);
        if (ret < 0)
            break;
        return;

    case UVC_EVENT_STREAMON:
        printf("UVC_EVENT_STREAMON received\n");
        if (!dev->bulk)
            uvc_handle_streamon_event(dev);
        return;

    case UVC_EVENT_STREAMOFF:
        printf("UVC_EVENT_STREAMOFF received\n");
        /* Stop V4L2 streaming... */
        if (!dev->run_standalone && SensorStartedStreaming) {
            /* UVC - V4L2 integrated path. */

            camDepthSensor->stop();
            SensorStartedStreaming = false;
        }

        /* ... and now UVC streaming.. */
        if (dev->is_streaming) {
            uvc_uninit_device(dev);
            uvc_video_stream(dev, 0);
            uvc_video_reqbufs(dev, 0);
            dev->is_streaming = 0;
            dev->first_buffer_queued = 0;
            eeprom_read_len = 0;
            eeprom_write_len = 0;
        }

        return;
    default:
        printf("UNKNOWN EVENT received\n");
    }

    // printf("Sending response to UVC event\n");
    ret = ioctl(dev->uvc_fd, UVCIOC_SEND_RESPONSE, &resp);
    if (ret < 0) {
        printf("UVCIOC_S_EVENT failed: %s (%d)\n", strerror(errno), errno);
        return;
    }
}

static void uvc_events_init(struct uvc_device *dev) {
    struct v4l2_event_subscription sub;
    unsigned int payload_size;

    switch (dev->fcc) {
    case V4L2_PIX_FMT_YUYV:
        payload_size = dev->width * dev->height * 2;
        break;
    case V4L2_PIX_FMT_MJPEG:
        payload_size = dev->imgsize;
        break;
    }

    uvc_fill_streaming_control(dev, &dev->probe, 0, 0);
    uvc_fill_streaming_control(dev, &dev->commit, 0, 0);

    if (dev->bulk) {
        /* FIXME Crude hack, must be negotiated with the driver. */
        dev->probe.dwMaxPayloadTransferSize =
            dev->commit.dwMaxPayloadTransferSize = payload_size;
    }

    memset(&sub, 0, sizeof sub);
    sub.type = UVC_EVENT_SETUP;
    ioctl(dev->uvc_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_DATA;
    ioctl(dev->uvc_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_STREAMON;
    ioctl(dev->uvc_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
    sub.type = UVC_EVENT_STREAMOFF;
    ioctl(dev->uvc_fd, VIDIOC_SUBSCRIBE_EVENT, &sub);
}

/* ---------------------------------------------------------------------------
 * main
 */

static void usage(const char *argv0) {
    fprintf(stderr, "Usage: %s [options]\n", argv0);
    fprintf(stderr, "Available options are\n");
    fprintf(stderr, " -b Use bulk mode\n");
    fprintf(stderr, " -d Do not use any real V4L2 capture device\n");
    fprintf(stderr, " -f <format>    Select frame format\n\t"
                    "0 = V4L2_PIX_FMT_YUYV\n\t"
                    "1 = V4L2_PIX_FMT_MJPEG\n");
    fprintf(stderr, " -h Print this help screen and exit\n");
    fprintf(stderr, " -i image	MJPEG image\n");
    fprintf(stderr, " -m Streaming mult for ISOC (b/w 0 and 2)\n");
    fprintf(stderr, " -n Number of Video buffers (b/w 2 and 32)\n");
    fprintf(stderr, " -o <IO method> Select UVC IO method:\n\t"
                    "0 = MMAP\n\t"
                    "1 = USER_PTR\n");
    fprintf(stderr, " -r <resolution> Select frame resolution:\n\t"
                    "0 = 360p, nHD (640x360)\n\t"
                    "1 = 720p, HD (1280x720)\n\t"
                    "2 = 960p, bVGA (640x960)\n\t"
                    "3 = 750p, bVGA (668x750)\n");
    fprintf(stderr, " -s <speed> Select USB bus speed (b/w 0 and 2)\n\t"
                    "0 = Full Speed (FS)\n\t"
                    "1 = High Speed (HS)\n\t"
                    "2 = Super Speed (SS)\n");
    fprintf(stderr, " -t Streaming burst (b/w 0 and 15)\n");
    fprintf(stderr, " -u device	UVC Video Output device\n");
    fprintf(stderr, " -v device	V4L2 Video Capture device\n");
    fprintf(stderr, " -a Indicate that this is TOF hw\n");
    fprintf(stderr,
            " -p <filename> name of fallback firmware file to be used\n");
}

int main(int argc, char *argv[]) {

    // Init google logging system
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;

    DLOG(INFO) << argv[0] << " "
               << "has started";

    signal(SIGINT, stopHandler);
    signal(SIGKILL, stopHandler);
    signal(SIGTERM, stopHandler);
    signal(SIGQUIT, stopHandler);

    struct uvc_device *udev;
    struct timeval tv;
    struct v4l2_format fmt;
    const char *uvc_devname = "/dev/video6";
    char *mjpeg_image = NULL;

    fd_set fdsv, fdsu;
    int ret, opt, nfds;
    int bulk_mode = 0;
    int dummy_data_gen_mode = 0;
    /* Frame format/resolution related params. */
    int default_format = 0;     /* V4L2_PIX_FMT_YUYV */
    int default_resolution = 0; /* HD 640p */
    int nbufs = 2;              /* Ping-Pong buffers */
    /* USB speed related params */
    int mult = 0;
    int burst = 0;
    enum usb_device_speed speed = USB_SPEED_HIGH; /* High-Speed */
    enum io_method uvc_io_method = IO_METHOD_USERPTR;

    DLOG(INFO) << "This UVC instance is using aditof sdk version: "
               << ADITOF_API_VERSION;

    auto sensorsEnumerator =
        aditof::SensorEnumeratorFactory::buildTargetSensorEnumerator();
    if (!sensorsEnumerator) {
        printf("Failed to construct a sensors enumerator!\n");
        return 1;
    }

    sensorsEnumerator->searchSensors();
    sensorsEnumerator->getDepthSensors(depthSensors);
    sensorsEnumerator->getStorages(storages);
    sensorsEnumerator->getTemperatureSensors(temperatureSensors);

    if (depthSensors.size() < 1) {
        printf("No camera sensor are available!\n");
        return 1;
    }

    camDepthSensor = depthSensors[0];
    sensorV4lBufAccess =
        std::dynamic_pointer_cast<aditof::V4lBufferAccessInterface>(
            camDepthSensor);
    if (!sensorV4lBufAccess) {
        printf("Camera sensor is not of type V4lBufferAccessInterface!\n");
        return 1;
    }
    std::string availableSensorsBlob;
    // Build a message about available sensors types to be sent to the UVC client
    aditof::SensorDetails camSensorDetails;
    camDepthSensor->getDetails(camSensorDetails);

    DLOG(INFO) << "Storages found:";
    int storage_id = 0;
    for (const auto &storage : storages) {
        std::string name;
        storage->getName(name);
        availableSensorsBlob += "STORAGE_NAME=" + name + ";";
        availableSensorsBlob +=
            "STORAGE_ID=" + std::to_string(storage_id) + ";";
        ++storage_id;
        DLOG(INFO) << name;
    }

    DLOG(INFO) << "Temperature sensors found:";
    int temp_sensor_id = 0;
    for (const auto &sensor : temperatureSensors) {
        std::string name;
        sensor->getName(name);
        availableSensorsBlob += "TEMP_SENSOR_NAME=" + name + ";";
        availableSensorsBlob +=
            "TEMP_SENSOR_ID=" + std::to_string(temp_sensor_id) + ";";
        ++temp_sensor_id;
        DLOG(INFO) << name;
    }

    DLOG(INFO) << "Message blob about available sensors to be sent to remote:";
    DLOG(INFO) << availableSensorsBlob;

    const uint32_t buffLen = availableSensorsBlob.length();
    sensorsInfoBuffer = new char[buffLen + sizeof(uint16_t)];
    uint16_t *buffSize = reinterpret_cast<uint16_t *>(sensorsInfoBuffer);
    *buffSize = buffLen;
    memcpy(sensorsInfoBuffer + sizeof(uint16_t), availableSensorsBlob.c_str(),
           buffLen);

    while ((opt = getopt(argc, argv, "abdf:hi:m:n:o:r:s:t:u:v:p:")) != -1) {
        switch (opt) {
        case 'a':
            ad903x_hw = 1;
            break;
        case 'b':
            bulk_mode = 1;
            break;

        case 'd':
            dummy_data_gen_mode = 1;
            break;

        case 'f':
            if (atoi(optarg) < 0 || atoi(optarg) > 1) {
                usage(argv[0]);
                return 1;
            }

            default_format = atoi(optarg);
            break;

        case 'h':
            usage(argv[0]);
            return 1;

        case 'i':
            mjpeg_image = optarg;
            break;

        case 'm':
            if (atoi(optarg) < 0 || atoi(optarg) > 2) {
                usage(argv[0]);
                return 1;
            }

            mult = atoi(optarg);
            printf("Requested Mult value = %d\n", mult);
            break;

        case 'n':
            if (atoi(optarg) < 2 || atoi(optarg) > 32) {
                usage(argv[0]);
                return 1;
            }

            nbufs = atoi(optarg);
            printf("Number of buffers requested = %d\n", nbufs);
            break;

        case 'o':
            if (atoi(optarg) < 0 || atoi(optarg) > 1) {
                usage(argv[0]);
                return 1;
            }

            uvc_io_method = static_cast<io_method>(atoi(optarg));
            printf("UVC: IO method requested is %s\n",
                   (uvc_io_method == IO_METHOD_MMAP) ? "MMAP" : "USER_PTR");
            break;

        case 'r':
            if (atoi(optarg) < 0 || atoi(optarg) > 3) {
                usage(argv[0]);
                return 1;
            }

            default_resolution = atoi(optarg);
            break;

        case 's':
            if (atoi(optarg) < 0 || atoi(optarg) > 2) {
                usage(argv[0]);
                return 1;
            }

            speed = static_cast<usb_device_speed>(atoi(optarg));
            break;

        case 't':
            if (atoi(optarg) < 0 || atoi(optarg) > 15) {
                usage(argv[0]);
                return 1;
            }

            burst = atoi(optarg);
            printf("Requested Burst value = %d\n", burst);
            break;

        case 'u':
            uvc_devname = optarg;
            break;

        case 'v':
            break;

        case 'p':
            strcpy(v4l2_subdev_prog_file, optarg);
            break;

        default:
            printf("Invalid option '-%c'\n", opt);
            usage(argv[0]);
            return 1;
        }
    }

    if (!dummy_data_gen_mode && !mjpeg_image) {
        /*
         * Try to set the default format at the V4L2 video capture
         * device as requested by the user.
         */
        CLEAR(fmt);
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        /* Seperating v4l2 resolution from UVC resolution, depending on HW,
         * this can be set to - TOF: 640x960 or D3: 1280x720 */
        if (ad903x_hw) {
            fmt.fmt.pix.width = default_resolution == 2 ? 640 : 668;
            fmt.fmt.pix.height = default_resolution == 2 ? 960 : 750;
        } else {
            fmt.fmt.pix.width = 1280;
            fmt.fmt.pix.height = 720;
        }

        fmt.fmt.pix.sizeimage =
            (default_format == 0)
                ? (fmt.fmt.pix.width * fmt.fmt.pix.height * 2)
                : (fmt.fmt.pix.width * fmt.fmt.pix.height * 1.5);
        fmt.fmt.pix.pixelformat =
            (default_format == 0) ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;

        // Open communication with depth sensor
        camDepthSensor->open();

        // Get depth sensor handle
        void *handle;
        camDepthSensor->getHandle(&handle);

        // Open communication with storages
        for (auto &storage : storages) {
            storage->open(handle);
        }

        // Open communication with temperature sensors
        for (auto &sensor : temperatureSensors) {
            sensor->open(handle);
        }
    }

    /* Open the UVC device. */
    ret = uvc_open(&udev, uvc_devname);
    if (udev == NULL || ret < 0)
        return 1;

    udev->uvc_devname = uvc_devname;

    printf("Set parameters by user\n");

    /* Set parameters as passed by user. */
    if (default_resolution == 0) {
        udev->width = 640;
        udev->height = 360;
    } else if (default_resolution == 1) {
        udev->width = 1280;
        udev->height = 720;
    } else if (default_resolution == 2) {
        udev->width = 640;
        udev->height = 960;
    } else /*if(default_resolution == 3)*/ {
        udev->width = 668;
        udev->height = 750;
    }

    udev->imgsize = (default_format == 0) ? (udev->width * udev->height * 2)
                                          : (udev->width * udev->height * 1.5);
    udev->fcc = (default_format == 0) ? V4L2_PIX_FMT_YUYV : V4L2_PIX_FMT_MJPEG;
    udev->io = uvc_io_method;
    udev->bulk = bulk_mode;
    udev->nbufs = nbufs;
    udev->mult = mult;
    udev->burst = burst;
    udev->speed = speed;

    if (dummy_data_gen_mode || mjpeg_image)
        /* UVC standalone setup. */
        udev->run_standalone = 1;

    ret = uvc_video_set_format(udev);
    if (ret < 0) {
        printf("Failed to set format for UVC\n");
    }

    if (!dummy_data_gen_mode && !mjpeg_image) {
        /* UVC - V4L2 integrated path */
        // vdev->nbufs = nbufs;

        /*
         * IO methods used at UVC and V4L2 domains must be
         * complementary to avoid any memcpy from the CPU.
         */
        switch (uvc_io_method) {
        case IO_METHOD_MMAP:
            // vdev->io = IO_METHOD_USERPTR;
            break;

        case IO_METHOD_USERPTR:
        default:
            // vdev->io = IO_METHOD_MMAP;
            break;
        }
    }

    printf("Set IO METHOD passed!\n");

    switch (speed) {
    case USB_SPEED_FULL:
        /* Full Speed. */
        if (bulk_mode)
            udev->maxpkt = 64;
        else
            udev->maxpkt = 1023;
        break;

    case USB_SPEED_HIGH:
        /* High Speed. */
        if (bulk_mode)
            udev->maxpkt = 512;
        else
            udev->maxpkt = 1024;
        break;

    case USB_SPEED_SUPER:
    default:
        /* Super Speed. */
        if (bulk_mode)
            udev->maxpkt = 1024;
        else
            udev->maxpkt = 1024;
        break;
    }

    std::vector<aditof::FrameDetails> frameDetailsVector;
    camDepthSensor->getAvailableFrameTypes(frameDetailsVector);

    camDepthSensor->setFrameType(frameDetailsVector.front());
    camDepthSensor->start();

    /* Init UVC events. */
    uvc_events_init(udev);

    int deviceFd = -1;
    sensorV4lBufAccess->getDeviceFileDescriptor(deviceFd);

    stop.store(false);

    eeprom_write_ready = true;

    while (!stop.load()) {

        FD_ZERO(&fdsu);
        FD_ZERO(&fdsv);

        /* We want both setup and data events on UVC interface.. */
        FD_SET(udev->uvc_fd, &fdsu);

        /* We want data events from the LocalDevice */
        FD_SET(deviceFd, &fdsv);

        fd_set efds = fdsu;
        fd_set dfds = fdsu;

        /* Timeout - allow for possibile delays in Host requests */
        tv.tv_sec = 10;
        tv.tv_usec = 0;

        nfds = max(udev->uvc_fd, deviceFd);
        ret = select(nfds + 1, &fdsv, &dfds, &efds, &tv);

        if (-1 == ret) {
            printf("select error %d, %s\n", errno, strerror(errno));
            if (EINTR == errno)
                continue;

            break;
        }

        if (0 == ret) {
            printf("select timeout\n");
            break;
        }

        if (FD_ISSET(udev->uvc_fd, &efds)) {
            // printf("uvc_event_process");
            uvc_events_process(udev);
        }

        if (FD_ISSET(udev->uvc_fd, &dfds)) {
            // printf("UVC VIDEO PROCESS called !!!!!");
            uvc_video_process(udev);
        }

        if (FD_ISSET(deviceFd, &fdsv)) {
            v4l2_process_data(udev);
        }
    }

    if (!dummy_data_gen_mode && !mjpeg_image && SensorStartedStreaming) {
        camDepthSensor->stop();
    }

    if (udev->is_streaming) {
        /* ... and now UVC streaming.. */
        uvc_video_stream(udev, 0);
        uvc_uninit_device(udev);
        uvc_video_reqbufs(udev, 0);
        udev->is_streaming = 0;
    }

    uvc_close(udev);

    if (eepromWriteThread.joinable())
        eepromWriteThread.join();

    // Close storages
    for (auto &storage : storages) {
        storage->close();
    }

    // Close temperature sensors
    for (auto &sensor : temperatureSensors) {
        sensor->close();
    }

    if (sensorsInfoBuffer) {
        delete[] sensorsInfoBuffer;
        sensorsInfoBuffer = nullptr;
    }

    DLOG(INFO) << argv[0] << " "
               << "is now closing gracefully";

    return 0;
}
