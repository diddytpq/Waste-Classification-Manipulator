import time
from absl import app, flags, logging
from absl.flags import FLAGS
import cv2
import tensorflow as tf
from yolov3_tf2.models import YoloV3, YoloV3Tiny
from yolov3_tf2.dataset import transform_images
from yolov3_tf2.utils import draw_outputs, location_convert, image_preprocess, location_convert_rgb, check_list_1, check_list_2, location2motor
from yolov3_tf2.inverse_kinematics import inverse_6
import numpy as np
import pyrealsense2 as rs
import serial 
from multiprocessing import Process, Pipe, Manager
from multiprocessing.managers import BaseManager

#python multi_process_realsense.py --classes ./data/custom_1.names --weights ./weight/waste_classification_darknet_90.tf --video 0 --num_classes 2

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

FLAGS.yolo_iou_threshold = 0.3
FLAGS.yolo_score_threshold = 0.7


class detect_location(object):
    def __init__(self):
        self.list = []

    def set(self, data_list):
        self.list = data_list

    def get(self):
        return self.list


def serial_trans(location):
    print('----------------thread start--------------------')
    ser_1 = serial.Serial('COM4', 9600) #윈도우 로봇팔 시리얼
    #ser_1 = serial.Serial('/dev/ttyACM0', 9600) #우분투 로봇팔 시리얼
    stop = '2'
    on = '0'
    rail_stop = stop.encode('utf-8')
    rail_on = on.encode('utf-8')
    time.sleep(1)
    print('----------------serial start--------------------')

    while True:
        location_list = location.get()

        if len(location_list)>0:
            location_list_2 = check_list_2(location_list)
            
            for i in range(len(location_list_2)):
                print("{}번째 x 좌표:{} y 좌표:{}, z 좌표:{}".format(i+1,location_list_2[i][0],location_list_2[i][1],location_list_2[i][2]))
                
            #print(location_list)

        if check_list_1(location_list) == True:
            ser_1.write(rail_stop) #레일 off
            print('rail off')
            time.sleep(1)
            location_list = check_list_2(location.get())

            while True:

                if len(location_list) == 0:
                    ser_1.write(rail_on) #레일 on
                    print('end')
                    print('rali_start')
                    break

                print(location_list)
                moter_list = inverse_6(location_list[0])
                string = location2motor(moter_list) # 좌표 자리수 맞게 조정

                print('로봇팔 모터 값 전송:\n motor_1: {}\n motor_2: {}\n motor_3: {}\n motor_4: {}\n motor_5: {}\n motor_6: {}'\
                        .format(moter_list[0],moter_list[1],moter_list[2],moter_list[3],moter_list[4],moter_list[5]))
                string = string.encode('utf-8')
                ser_1.write(string) #로봇팔 좌표전송

                while True:

                    if ser_1.readable():
                        print("done")
                        location_list = check_list_2(location.get())
                        break
                    #time.sleep(5)
        


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

    times = []

    BaseManager.register('detect_location', detect_location)
    manager = BaseManager()
    manager.start()
    inst = manager.detect_location()

    thread = Process(target=serial_trans, args=[inst])
    thread.start()

    pipeline=rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            logging.warning("Empty Frame")
            time.sleep(0.1)
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.5), cv2.COLORMAP_JET)

        img_in = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        img_in = tf.expand_dims(img_in, 0)
        img_in = transform_images(img_in, FLAGS.size)


        t1 = time.time()
        boxes, scores, classes, nums = yolo.predict(img_in)
        t2 = time.time()
        times.append(t2-t1)
        times = times[-20:]

        img, location = draw_outputs(color_image,(boxes, scores, classes, nums), class_names,depth_colormap)

        img = cv2.putText(color_image, "Time: {:.2f}ms".format(sum(times)/len(times)*1000), (0, 30),
                          cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 0, 255), 2)

        inst.set(location)

        
        cv2.imshow('output', color_image)
        cv2.imshow('depth',depth_colormap)
        key=cv2.waitKey(1) 
        if key == 27:
            cv2.destroyAllWindows()
            pipeline.stop()
            break
        
            

    #vid.release()
    


if __name__ == '__main__':
    try:
        app.run(main)
    except SystemExit:
        pass