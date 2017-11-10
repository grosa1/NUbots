#!/usr/bin/env python3

from nuclear import Reactor, on, Trigger, Single, With, Every
from message.vision import ReprojectedImage, Obstacle
import numpy as np
import tensorflow as tf
import yaml
import datetime
import time

#from kernels import *

#patterns = {0x42474752: 0, 0x47425247: 1, 0x47524247: 2, 0x52474742: 3}

@Reactor
class PedestrianDetector(object):

    def __init__(self):
        # Constructor for PedestrianDetector
        self.config = yaml.load(open('config/PedestrianDetector.yaml', 'r'))

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            self.od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.config['model']['checkpoint_file'], 'rb') as fid:
                serialized_graph = fid.read()
                self.od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(self.od_graph_def, name='')

        self.avg_fp_ms = 0
        self.avg_count = 0

    # @on(Configuration('PedestrianDetector.yaml'))
    # def PedestrianDetector_configfuration(self, config):
    #     # Use configuration here from file PedestrianDetector.yaml

    @on(Trigger(ReprojectedImage), Single())
    def run_detection(self, image):
        # Convert image to numpy array
        image_np = np.zeros((image.dimensions[1], image.dimensions[0], 3), dtype = np.uint8)
        img = np.array(image.data).reshape((image.dimensions[1], image.dimensions[0], 3)).astype(np.uint8)

        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                # Definite input and output Tensors for detection_graph
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

                # Each box represents a part of the image where a particular object was detected.
                detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image_np, axis = 0)

                # Actual detection.
                detection_start = time.time()
                (boxes, scores, classes, num) = sess.run(
                        [detection_boxes, detection_scores, detection_classes, num_detections],
                        feed_dict={image_tensor: image_np_expanded})
                detection_end = time.time()
                self.avg_fp_ms += detection_end - detection_start
                self.avg_count += 1
                print('PedestrianDetector::run_detection: Detection time: {0:.4f} s (avg: {1:.4f} s)'.format(
                                                detection_end - detection_start, self.avg_fp_ms / self.avg_count))

                msg = []
                merged_boxes = self.non_max_suppression_fast(np.squeeze(boxes), 0.5)
                for box in merged_boxes:
                    obj = Obstacle()
                    obj.visObject.timestamp = datetime.datetime.now()
                    obj.visObject.camera_id = image.camera_id
                    ymin, xmin, ymax, xmax = box.tolist()
                    obj.shape.points = [[int(xmin * image.dimensions[0]), int(ymin * image.dimensions[1])],
                                        [int(xmax * image.dimensions[0]), int(ymin * image.dimensions[1])],
                                        [int(xmax * image.dimensions[0]), int(ymax * image.dimensions[1])],
                                        [int(xmin * image.dimensions[0]), int(ymax * image.dimensions[1])]]
                    obj.team = Obstacle.Team.UNKNOWN_TEAM

                    msg.append(obj)

                self.emit(msg)

    # https://stackoverflow.com/a/37850394
    def non_max_suppression_fast(self, boxes, overlap_threshold):
        # if there are no boxes, return an empty list
        if len(boxes) == 0:
            return []

        # initialize the list of picked indexes
        pick = []

        # grab the coordinates of the bounding boxes
        y1 = boxes[:, 0]
        x1 = boxes[:, 1]
        y2 = boxes[:, 2]
        x2 = boxes[:, 3]

        # compute the area of the bounding boxes and sort the bounding
        # boxes by the bottom-right y-coordinate of the bounding box
        area = (x2 - x1 + 1) * (y2 - y1 + 1)
        idxs = np.argsort(y2)

        # keep looping while some indexes still remain in the indexes
        # list
        while len(idxs) > 0:
            # grab the last index in the indexes list and add the index value to the list of picked indexes
            last = len(idxs) - 1
            i = idxs[last]
            pick.append(i)

            # find the largest (x, y) coordinates for the start of the bounding box and the smallest (x, y) coordinates
            # for the end of the bounding box
            xx1 = np.maximum(x1[i], x1[idxs[:last]])
            yy1 = np.maximum(y1[i], y1[idxs[:last]])
            xx2 = np.minimum(x2[i], x2[idxs[:last]])
            yy2 = np.minimum(y2[i], y2[idxs[:last]])

            # compute the width and height of the bounding box
            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)

            # compute the ratio of overlap
            overlap = (w * h) / area[idxs[:last]]

            # delete all indexes from the index list that have
            idxs = np.delete(idxs, np.concatenate(([last], np.where(overlap > overlap_threshold)[0])))

        # return only the bounding boxes that were picked using the integer data type
        return boxes[pick]
