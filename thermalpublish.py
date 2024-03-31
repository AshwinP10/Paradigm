#!/usr/bin/env python
# -*- coding: utf-8 -*-
from flask import Flask, render_template, Response
from uvctypes import *
import time
import cv2
import numpy as np
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
import platform
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

BUF_SIZE = 2
q = Queue(BUF_SIZE)
app = Flask(__name__)
bridge = CvBridge()
publisher = None  # ROS publisher variable

def py_frame_callback(frame, userptr):
    array_pointer = cast(frame.contents.data, POINTER(c_uint16 * (frame.contents.width * frame.contents.height)))
    data = np.frombuffer(
        array_pointer.contents, dtype=np.dtype(np.uint16)
    ).reshape(
        frame.contents.height, frame.contents.width
    )  # no copy

    if frame.contents.data_bytes != (2 * frame.contents.width * frame.contents.height):
        return

    if not q.full():
        q.put(data)

    # Publish ROS message here
    if publisher is not None:
        img_msg = bridge.cv2_to_imgmsg(data, encoding="mono16")
        publisher.publish(img_msg)

PTR_PY_FRAME_CALLBACK = CFUNCTYPE(None, POINTER(uvc_frame), c_void_p)(py_frame_callback)

def ktof(val):
    return (1.8 * ktoc(val) + 32.0)

def ktoc(val):
    return (val - 27315) / 100.0

def raw_to_8bit(data):
    cv2.normalize(data, data, 0, 65535, cv2.NORM_MINMAX)
    np.right_shift(data, 8, data)
    return cv2.cvtColor(np.uint8(data), cv2.COLOR_GRAY2RGB)

def display_temperature(img, val_k, loc, color):
    val = ktof(val_k)
    cv2.putText(img, "{0:.1f} degF".format(val), loc, cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
    x, y = loc
    cv2.line(img, (x - 2, y), (x + 2, y), color, 1)
    cv2.line(img, (x, y - 2), (x, y + 2), color, 1)

@app.before_first_request
def before_start():
    ctx = POINTER(uvc_context)()
    dev = POINTER(uvc_device)()
    global devh
    devh = POINTER(uvc_device_handle)()
    ctrl = uvc_stream_ctrl()

    res = libuvc.uvc_init(byref(ctx), 0)
    if res < 0:
        print("uvc_init error")
        exit(1)

    res = libuvc.uvc_find_device(ctx, byref(dev), PT_USB_VID, PT_USB_PID, 0)
    if res < 0:
        print("uvc_find_device error")
        exit(1)

    res = libuvc.uvc_open(dev, byref(devh))
    if res < 0:
        print("uvc_open error")
        exit(1)

    print("device opened!")

    print_device_info(devh)
    print_device_formats(devh)

    set_manual_ffc(devh)
    print_shutter_info(devh)

    frame_formats = uvc_get_frame_formats_by_guid(devh, VS_FMT_GUID_Y16)
    if len(frame_formats) == 0:
        print("device does not support Y16")
        exit(1)

    libuvc.uvc_get_stream_ctrl_format_size(devh, byref(ctrl), UVC_FRAME_FORMAT_Y16,
                                           frame_formats[0].wWidth, frame_formats[0].wHeight,
                                           int(1e7 / frame_formats[0].dwDefaultFrameInterval))

    res = libuvc.uvc_start_streaming(devh, byref(ctrl), PTR_PY_FRAME_CALLBACK, None, 0)
    if res < 0:
        print("uvc_start_streaming failed: {0}".format(res))
        exit(1)

##gen frame
def generate_frames():
    pasttime = int(time.time())
    print('hi1')
    while True:
        curr_time = int(time.time())
        if (curr_time - pasttime) >= 180:
            perform_manual_ffc(devh)
            pasttime = curr_time
        data = q.get(True, 500)
        if data is None:
            break
        data = cv2.resize(data[:, :], (640, 480))
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(data)
        img = raw_to_8bit(data)
        display_temperature(img, minVal, minLoc, (255, 0, 0))
        display_temperature(img, maxVal, maxLoc, (0, 0, 255))
        img_rgb = cv2.applyColorMap(img, cv2.COLORMAP_JET)
        ret, buffer = cv2.imencode('.jpg', img_rgb)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/ffc')
def man_ffc():
    perform_manual_ffc(devh)
    return 'Performed FFC'

if __name__ == '__main__':
    # Initialize ROS node and publisher
    rospy.init_node('thermal_camera_publisher', anonymous=True)
    publisher = rospy.Publisher('/thermal_camera/image', Image, queue_size=10)
    
    app.run(host='0.0.0.0', debug=True, port="5000")
