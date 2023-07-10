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


def train():
    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
    cfg.DATASETS.TRAIN = ("mydata",)
    cfg.DATASETS.TEST = ()  # no metrics implemented for this dataset
    cfg.DATALOADER.NUM_WORKERS = 2
    cfg.MODEL.WEIGHTS = "detectron2://COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl"  # initialize from model zoo
    # cfg.MODEL.WEIGHTS = "./output/model_final.pth"  # initialize from model zoo
    cfg.SOLVER.IMS_PER_BATCH = 2
    cfg.SOLVER.BASE_LR = 0.0025
    cfg.SOLVER.MAX_ITER = (2500)  # 300 iterations seems good enough, but you can certainly train longer
    cfg.MODEL.ROI_HEADS.BATCH_SIZE_PER_IMAGE = (128)  # faster, and good enough for this toy dataset
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1  # 2 classes (zao,jin)
    os.makedirs(cfg.OUTPUT_DIR, exist_ok=True)
    trainer = DefaultTrainer(cfg)
    trainer.resume_or_load(resume=False)
    trainer.train()

def predict():
    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
    cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, "model_final.pth")
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5   # set the testing threshold for this model
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
    cfg.DATASETS.TEST = ("mydata", )
    predictor = DefaultPredictor(cfg)

    data_f = './val/12.jpg'
    im = cv2.imread(data_f)
    outputs = predictor(im)
    v = Visualizer(im[:, :, ::-1], metadata=mydata_metadata, scale=0.8, instance_mode=ColorMode.IMAGE_BW) # remove the colors of unsegmented pixels
    v = v.draw_instance_predictions(outputs["instances"].to("cpu"))
    img = v.get_image()[:, :, ::-1]
    cv2.imshow('rr', img)
    cv2.waitKey(0)

def vis_data():
    dataset_dicts = DatasetCatalog.get("mydata")
    for d in random.sample(dataset_dicts, 4):
        # d=dataset_dicts[0]
        img = cv2.imread(d["file_name"])
        print(img.shape)
        visualizer = Visualizer(img[:, :, ::-1], metadata=mydata_metadata, scale=1)
        vis = visualizer.draw_dataset_dict(d)
        img = vis.get_image()[:, :, ::-1]
        cv2.imshow('rr', img)
        cv2.waitKey(0)


if __name__ == '__main__':
    try:
        register_coco_instances("mydata", {}, "./ccjson/anno.json", "./")
        mydata_metadata = MetadataCatalog.get("mydata")
        # train()
        predict()
        # vis_data()
    except Exception:
        pass
