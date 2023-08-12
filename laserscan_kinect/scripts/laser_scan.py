#!/usr/bin/env python3

import cv2
import numpy as np
import depthai as dai
import argparse
from matplotlib import pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument(
    "-res",
    "--resolution",
    type=str,
    default="720",
    help="Sets the resolution on mono cameras. Options: 800 | 720 | 400",
)
parser.add_argument(
    "-lr",
    "--lrcheck",
    default=False,
    action="store_true",
    help="Better handling for occlusions",
)
parser.add_argument(
    "-e",
    "--extended",
    default=False,
    action="store_true",
    help="Closer-in minimum depth, disparity range is doubled",
)
parser.add_argument(
    "-s",
    "--subpixel",
    default=False,
    action="store_true",
    help="Better accuracy for longer distance, fractional disparity 32-levels",
)
parser.add_argument(
    "-m",
    "--median",
    type=str,
    default="7x7",
    help="Choose the size of median filtering. Options: OFF | 3x3 | 5x5 | 7x7 (default)",
)
parser.add_argument(
    "-swlr",
    "--swap_left_right",
    default=False,
    action="store_true",
    help="Swap left right frames",
)
parser.add_argument(
    "-a",
    "--alpha",
    type=float,
    default=None,
    help="Alpha scaling parameter to increase FOV",
)
args = parser.parse_args()


def convert_to_laserscan(depth_frame, intrinsics):
    

    pass


RES_MAP = {
    '800': {'w': 1280, 'h': 800, 'res': dai.MonoCameraProperties.SensorResolution.THE_800_P },
    '720': {'w': 1280, 'h': 720, 'res': dai.MonoCameraProperties.SensorResolution.THE_720_P },
    '400': {'w': 640, 'h': 400, 'res': dai.MonoCameraProperties.SensorResolution.THE_400_P }
}
if args.resolution not in RES_MAP:
    exit("Unsupported resolution!")

resolution = RES_MAP[args.resolution]

lrcheck = args.lrcheck  # Better handling for occlusions
extended = args.extended  # Closer-in minimum depth, disparity range is doubled
subpixel = args.subpixel  # Better accuracy for longer distance, fractional disparity 32-levels

medianMap = {
    "OFF": dai.StereoDepthProperties.MedianFilter.MEDIAN_OFF,
    "3x3": dai.StereoDepthProperties.MedianFilter.KERNEL_3x3,
    "5x5": dai.StereoDepthProperties.MedianFilter.KERNEL_5x5,
    "7x7": dai.StereoDepthProperties.MedianFilter.KERNEL_7x7,
}
if args.median not in medianMap:
    exit("Unsupported median size!")

median = medianMap[args.median]

print("StereoDepth config options:")
print(f"    Resolution:  {resolution['w']}x{resolution['h']}")
print("    Left-Right check:  ", lrcheck)
print("    Extended disparity:", extended)
print("    Subpixel:          ", subpixel)
print("    Median filtering:  ", median)

device = dai.Device()
calibData = device.readCalibration()
print("Creating Stereo Depth pipeline")
pipeline = dai.Pipeline()

camLeft = pipeline.create(dai.node.MonoCamera)
camRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
xoutDepth = pipeline.create(dai.node.XLinkOut)

xoutRectifRight = pipeline.create(dai.node.XLinkOut)

if args.swap_left_right:
    camLeft.setCamera("right")
    camRight.setCamera("left")
else:
    camLeft.setCamera("left")
    camRight.setCamera("right")

for monoCam in (camLeft, camRight):  # Common config
    monoCam.setResolution(resolution['res'])
    # monoCam.setFps(20.0)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.initialConfig.setMedianFilter(median)  # KERNEL_7x7 default
stereo.setRectifyEdgeFillColor(0)  # Black, to better see the cutout
stereo.setLeftRightCheck(lrcheck)
stereo.setExtendedDisparity(extended)
stereo.setSubpixel(subpixel)
if args.alpha is not None:
    stereo.setAlphaScaling(args.alpha)
    config = stereo.initialConfig.get()
    config.postProcessing.brightnessFilter.minBrightness = 0
    stereo.initialConfig.set(config)

xoutDepth.setStreamName("depth")
xoutRectifRight.setStreamName("rectifiedRight")

camLeft.out.link(stereo.left)
camRight.out.link(stereo.right)
stereo.rectifiedRight.link(xoutRectifRight.input)
stereo.depth.link(xoutDepth.input)


cvColorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
cvColorMap[0] = [0, 0, 0]
print("Creating DepthAI device")
with device:
    device.startPipeline(pipeline)
    right_intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, resolution['w'], resolution['h'])
    # Create a receive queue for each stream
    qDepthList = device.getOutputQueue("depth", 8, blocking=False)
    qRectifiedRightList = device.getOutputQueue("rectifiedRight", 8, blocking=False)

    while True:
        depth = qDepthList.get().getFrame()
        right_rect = qRectifiedRightList.get().getCvFrame()
        cv2.imshow("depth", depth)

        # Blend the right_rect and depth image
        # depthColor = cv2.normalize(depth, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        # print(f'Max of depth is {np.max(depth)}')
        # print(f'Min of depth is {np.min(depth)}')
        # depthColor = cv2.convertScaleAbs(depth, alpha=255.0 / 65535.0)
        # print(f'Max of depthColor is {np.max(depthColor)}')
        # print(f'Min of depthColor is {np.min(depthColor)}')
        # print(depthColor.dtype)

        depthColor = cv2.convertScaleAbs(depth, 1, 255)
        depthColor = cv2.applyColorMap(depthColor, cv2.COLORMAP_JET)
        right_rect_col = cv2.cvtColor(right_rect, cv2.COLOR_GRAY2RGB)
        blended_image = cv2.addWeighted(right_rect_col, 0.5, depthColor, 0.5, 0.0)

        cv2.imshow("depth blended", blended_image)
        if cv2.waitKey(1) == ord("q"):
            break
