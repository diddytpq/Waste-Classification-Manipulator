import time
from absl import app, flags, logging
from absl.flags import FLAGS
import cv2
import tensorflow as tf
from yolov3_tf2.models import YoloV3, YoloV3Tiny
from yolov3_tf2.dataset import transform_images
from yolov3_tf2.utils import draw_outputs_ori, location_convert, image_preprocess, location_convert_rgb, check_list
import numpy as np
import serial
from multiprocessing import Process, Pipe, Manager
from multiprocessing.managers import BaseManager

#python multi_process_rgb.py --classes ./data/custom_1.names --weights ./weight/multi_9_5.tf --video 0 --num_classes 2

flags.DEFINE_string('classes', './data/coco.names', 'path to classes file')
flags.DEFINE_string('weights', './checkpoints/yolov3.tf',
                    'path to weights file')
flags.DEFINE_boolean('tiny', False, 'yolov3 or yolov3-tiny')
flags.DEFINE_integer('size', 416, 'resize images to')
flags.DEFINE_string('video', './data/video.mp4',
                    'path to video file or number for webcam)')
flags.DEFINE_string('output', None, 'path to output video')
flags.DEFINE_string('output_format', 'XVID', 'codec used in VideoWriter when saving video to file')
flags.DEFINE_integer('num_classes', 80, 'number of classes in the model')

FLAGS.yolo_iou_threshold = 0.5
FLAGS.yolo_score_threshold = 0.4

#ser_1 = serial.Serial('COM4', 9600) #로봇팔 시리얼
#ser_2 = serial.Serial('COM7', 9600) #레일 시리얼

class detect_location(object):
    def __init__(self):
        self.list = []

    def set(self, data_list):
        self.list = data_list

    def get(self):
        return self.list


def serial_trans(location):
    print('thread start')
    #ser_1 = serial.Serial('COM4', 9600) #로봇팔 시리얼
    #ser_2 = serial.Serial('COM7', 9600) #레일 시리얼
    stop='1'
    rail_stop = stop.encode('utf-8')
    time.sleep(1)
    print('serial start')

    while True:
        location_list = location.get()
        if len(location_list)>0:
            print(location_list)

        if check_list(location_list) == True:
            #ser_2.write(rail_stop) #레일 off
            print('rail off')
            time.sleep(1)
            location_list = location.get()
            while True:
                if len(location_list) == 0:
                    #ser_2.write(rail_stop) #레일 on
                    print('end')
                    print('rali_start')
                    break
                print(location_list)
                string = location_convert_rgb(location_list[0]) # 좌표 자리수 맞게 조정
            
                #string = string.encode('utf-8')
                #ser_1.write(string) #로봇팔 좌표전송
                print('로봇팔 좌표 전송:' + string)
                while True:
                    time.sleep(5)
                    location_list = location.get()
                    break



def main(_argv):

    physical_devices = tf.config.experimental.list_physical_devices('GPU')
    if len(physical_devices) > 0:
        tf.config.experimental.set_memory_growth(physical_devices[0], True)

    if FLAGS.tiny:
        yolo = YoloV3Tiny(classes=FLAGS.num_classes)
    else:
        yolo = YoloV3(classes=FLAGS.num_classes)

    yolo.load_weights(FLAGS.weights)
    logging.info('weights loaded')

    class_names = [c.strip() for c in open(FLAGS.classes).readlines()]
    logging.info('classes loaded')

    BaseManager.register('detect_location', detect_location)
    manager = BaseManager()
    manager.start()
    inst = manager.detect_location()

    thread = Process(target=serial_trans, args=[inst])
    thread.start()
    

    times = []

    try:
        vid = cv2.VideoCapture(int(FLAGS.video))
    except:
        vid = cv2.VideoCapture(FLAGS.video)

    out = None

    if FLAGS.output:
        # by default VideoCapture returns float instead of int
        width = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(vid.get(cv2.CAP_PROP_FPS))
        #codec = cv2.VideoWriter_fourcc(*FLAGS.output_format)
        #out = cv2.VideoWriter(FLAGS.output, codec, fps, (width, height))

    while True:
        _, img = vid.read()

        if img is None:
            logging.warning("Empty Frame")
            time.sleep(0.1)
            continue
        

        img_in = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        img_in = tf.expand_dims(img_in, 0)
        img_in = transform_images(img_in, FLAGS.size)

        t1 = time.time()
        boxes, scores, classes, nums = yolo.predict(img_in)
        t2 = time.time()
        times.append(t2-t1)
        times = times[-20:]

        img,location = draw_outputs_ori(img, (boxes, scores, classes, nums), class_names)      
        inst.set(location)
            

        img = cv2.putText(img, "Time: {:.2f}ms".format(sum(times)/len(times)*1000), (0, 30),
                          cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 2)
        fx,fy=1,1

        #img=cv2.resize(img,None,fx,fy)

        cv2.imshow('output', img)
        if cv2.waitKey(1) == ord('q'):break
            

    vid.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    try:
        app.run(main)
    except SystemExit:
        pass


