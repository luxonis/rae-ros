from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import depthai as dai
labelMap = [
            "person",
            "bicycle",
            "car",
            "motorbike",
            "aeroplane",
            "bus",
            "train",
            "truck",
            "boat",
            "traffic light",
            "fire hydrant",
            "stop sign",
            "parking meter",
            "bench",
            "bird",
            "cat",
            "dog",
            "horse",
            "sheep",
            "cow",
            "elephant",
            "bear",
            "zebra",
            "giraffe",
            "backpack",
            "umbrella",
            "handbag",
            "tie",
            "suitcase",
            "frisbee",
            "skis",
            "snowboard",
            "sports ball",
            "kite",
            "baseball bat",
            "baseball glove",
            "skateboard",
            "surfboard",
            "tennis racket",
            "bottle",
            "wine glass",
            "cup",
            "fork",
            "knife",
            "spoon",
            "bowl",
            "banana",
            "apple",
            "sandwich",
            "orange",
            "broccoli",
            "carrot",
            "hot dog",
            "pizza",
            "donut",
            "cake",
            "chair",
            "sofa",
            "pottedplant",
            "bed",
            "diningtable",
            "toilet",
            "tvmonitor",
            "laptop",
            "mouse",
            "remote",
            "keyboard",
            "cell phone",
            "microwave",
            "oven",
            "toaster",
            "sink",
            "refrigerator",
            "book",
            "clock",
            "vase",
            "scissors",
            "teddy bear",
            "hair drier",
            "toothbrush",
        ]
@dataclass(frozen=True)
class Detections:
    target: Optional[dai.ImgDetection] = None
    other: List[dai.ImgDetection] = field(default_factory=list)


def create_detection_metadata(detections, width, height):
    def build_detection_metadata(detection, color):
        return {
            "bbox": [
                detection.xmin * width,
                detection.ymin * height,
                detection.xmax * width,
                detection.ymax * height,
            ],
            "label": f"{labelMap[detection.label]} ({int(detection.confidence * 10000) / 100}%)",
            "color": color,
        }

    
    metadata = {
        "platform": "robothub",
        "frame_shape": [width, height],
        "config": {
            "output": {"img_scale": 1.0, "show_fps": True, "clickable": True},
            "detection": {
                "thickness": 1,
                "fill_transparency": 0.05,
                "box_roundness": 0,
                "color": [0, 255, 255],
                "bbox_style": 0,
                "line_width": 0.5,
                "line_height": 0.5,
                "hide_label": False,
                "label_position": 0,
                "label_padding": 10,
            },
            "text": {
                "font_color": [255, 255, 0],
                "font_transparency": 0.5,
                "font_scale": 1.0,
                "font_thickness": 2,
                "bg_transparency": 0.5,
                "bg_color": [0, 0, 0],
            },
        },
        "objects": [
            {
                "type": "detections",
                "detections": [build_detection_metadata(detection, [0, 255, 255]) for detection in detections.other]
                + ([build_detection_metadata(detections.target, [255, 0, 0])] if detections.target else []),
            }
        ],
    }
    return metadata
        
