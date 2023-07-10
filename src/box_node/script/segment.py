import random
import cv2
import os
from detectron2.utils.visualizer import Visualizer
from detectron2.data.catalog import MetadataCatalog, DatasetCatalog
from detectron2.engine import DefaultTrainer
from detectron2.config import get_cfg
from detectron2.utils.logger import setup_logger
setup_logger()
from detectron2 import model_zoo
from detectron2.engine.defaults import DefaultPredictor
from detectron2.utils.visualizer import ColorMode
from detectron2.data.datasets import register_coco_instances

def segment_2D(img):
    register_coco_instances("mydata", {}, "./ccjson/anno.json", "./")
    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
    cfg.MODEL.WEIGHTS = os.path.join("/home/mo/catkin_ws/src/box_node/sysmodel/output", "model_final.pth")
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5   # set the testing threshold for this model
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
    cfg.DATASETS.TEST = ("mydata", )
    predictor = DefaultPredictor(cfg)
    outputs = predictor(img)
    masks = outputs['instances'].get('pred_masks').cpu().numpy()
    return masks