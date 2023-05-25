#!/usr/bin/env python
#coding=utf-8
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import sys
import argparse
import logging
import time
from pathlib import Path
import glob
import json
import time


import numpy as np
from tqdm import tqdm
import cv2
import yaml

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from edgetpumodel import EdgeTPUModel
from utils import resize_and_pad, get_image_tensor, save_one_json, coco80_to_coco91_class

class aaImage:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/usb_cam/image_raw",Image, self.OriginImage)
    def OriginImage(self,msg):
        self.originimg = self.bridge.imgmsg_to_cv2(msg,"bgr8")




if __name__ == "__main__":
 
    # parser = argparse.ArgumentParser("EdgeTPU test runner")
    # parser.add_argument("--model", "-m", help="weights file", default='/home/aa/yolov5/runs/train/exp43/weights/best-int8_edgetpu.tflite')
    # parser.add_argument("--bench_speed", action='store_true', help="run speed test on dumm v y data")
    # parser.add_argument("--bench_image", action='store_true', help="run detection test")
    # parser.add_argument("--conf_thresh", type=float, default=0.25, help="model confidence threshold")
    # parser.add_argument("--iou_thresh", type=float, default=0.75, help="NMS IOU threshold")
    # parser.add_argument("--names", type=str, default='/home/aa/yolov5/data/targetdata.yaml', help="Names file")
    # parser.add_argument("--image", "-i", type=str, help="Image file to run detection on")
    # parser.add_argument("--device", type=int, default=0, help="Image capture device to run live detection")
    # parser.add_argument("--stream", action='store_true',default=True, help="Process a stream")
    # parser.add_argument("--bench_coco", action='store_true', help="Process a stream")
    # parser.add_argument("--coco_path", type=str, help="Path to COCO 2017 Val folder")
    # parser.add_argument("--quiet","-q", action='store_true', help="Disable logging (except errors)")

    model_add = '/home/iclab/Desktop/MAR/src/strategy/Kidsize_HuroCup/47/best-int8_edgetpu.tflite'
    conf_thresh = 0.7                                                                           #信心閥值
    iou_thresh = 0.0001                                                                         #真實框與檢測框重疊度
    yaml_add = '/home/iclab/Desktop/MAR/src/strategy/Kidsize_HuroCup/targetdata320.yaml'
    quiet = False

    # args = parser.parse_args()
    rospy.init_node('edge_yolo', anonymous=True)
    ros_image = aaImage()
    
    if quiet:
        logging.disable(logging.CRITICAL)
        logger.disabled = True
    
    # if args.stream and args.image:
    #     logger.error("Please select either an input image or a stream")
    #     exit(1)
    
    model = EdgeTPUModel(model_add, yaml_add, conf_thresh=conf_thresh, iou_thresh=iou_thresh)
    input_size = model.get_image_size()

    x = (255*np.random.random((3,*input_size))).astype(np.uint8)

    model.forward(x)

    # conf_thresh = 0.3
    # iou_thresh = 0.75
    # classes = None
    # agnostic_nms = False
    # max_det = 1000

    while not rospy.is_shutdown():    
        try:
            full_image, net_image, pad = get_image_tensor(ros_image.originimg, input_size[0])
            pred = model.forward(net_image)
                    
            model.process_predictions(pred[0], full_image, pad)
                    
            tinference, tnms = model.get_last_inference_time()
            FPS = 1/(tinference+tnms)

        except rospy.ROSInterruptException:
            pass

    # if args.bench_speed:
    #     logger.info("Performing test run")
    #     n_runs = 100
        
        
    #     inference_times = []
    #     nms_times = []
    #     total_times = []
        
    #     for i in tqdm(range(n_runs)):
    #         x = (255*np.random.random((3,*input_size))).astype(np.float32)
            
    #         pred = model.forward(x)
    #         tinference, tnms = model.get_last_inference_time()
            
    #         inference_times.append(tinference)
    #         nms_times.append(tnms)
    #         total_times.append(tinference + tnms)
            
    #     inference_times = np.array(inference_times)
    #     nms_times = np.array(nms_times)
    #     total_times = np.array(total_times)
            
    #     logger.info("Inference time (EdgeTPU): {:1.2f} +- {:1.2f} ms".format(inference_times.mean()/1e-3, inference_times.std()/1e-3))
    #     logger.info("NMS time (CPU): {:1.2f} +- {:1.2f} ms".format(nms_times.mean()/1e-3, nms_times.std()/1e-3))
    #     fps = 1.0/total_times.mean()
    #     logger.info("Mean FPS: {:1.2f}".format(fps))

    # elif args.bench_image:
    #     logger.info("Testing on Zidane image")
    #     model.predict("./data/images/zidane.jpg")

    # elif args.bench_coco:
    #     logger.info("Testing on COCO dataset")
        
    #     model.conf_thresh = 0.001
    #     model.iou_thresh = 0.65
        
    #     coco_glob = os.path.join(args.coco_path, "*.jpg")
    #     images = glob.glob(coco_glob)
        
    #     logger.info("Looking for: {}".format(coco_glob))
    #     ids = [int(os.path.basename(i).split('.')[0]) for i in images]
        
    #     out_path = "./coco_eval"
    #     os.makedirs("./coco_eval", exist_ok=True)
        
    #     logger.info("Found {} images".format(len(images)))
        
    #     class_map = coco80_to_coco91_class()
        
    #     predictions = []
        
    #     for image in tqdm(images):
    #         res = model.predict(image, save_img=False, save_txt=False)
    #         save_one_json(res, predictions, Path(image), class_map)
            
    #     pred_json = os.path.join(out_path,
    #                 "{}_predictions.json".format(os.path.basename(args.model)))
        
    #     with open(pred_json, 'w') as f:
    #         json.dump(predictions, f,indent=1)
        
    # elif args.image is not None:
    #     logger.info("Testing on user image: {}".format(args.image))
    #     model.predict(args.image)
        
    # elif args.stream:
    #     logger.info("Opening stream on device: {}".format(args.device))
        
    #     # cam = cv2.VideoCapture(args.device)
    #     time.sleep(0.2)    
    #     while True:
    #         # r = rospy.Rate(100)
    #         try:
    #         #     res, image = cam.read()
                
    #         #     if res is False:
    #         #         logger.error("Empty image received")
    #         #         break
    #         #     else:
    #             full_image, net_image, pad = get_image_tensor(ros_image.originimg, input_size[0])
    #             pred = model.forward(net_image)
                
    #             model.process_predictions(pred[0], full_image, pad)
                
    #             tinference, tnms = model.get_last_inference_time()
    #             # logger.info("Frame done in {}".format(tinference+tnms))
    #             print("Frame done in {}".format(tinference+tnms))
    #             # cv2.imshow("vedio",ros_image.originimg)
    #             # r.sleep()
    #             # time.sleep(0.2)

    #             # if cv2.waitKey(1)==ord('q'):
    #             #     break

    #         except KeyboardInterrupt:
    #             break
          
        # cam.release()

    
            
        

    

