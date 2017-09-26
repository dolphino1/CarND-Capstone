import rospy
import cv2
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import os
import time

from styx_msgs.msg import TrafficLight
from utils import label_map_util
from utils import visualization_utils as vis_util


class TLClassifier(object):
    def __init__(self, min_score_thresh=0.5):
        #TODO load classifier
	rospy.loginfo("loading light classification model...")
	cwd = os.path.dirname(os.path.realpath(__file__))
	
	frozen_model_path = rospy.get_param('/tl_classifier_model_path')

	label_map = label_map_util.load_labelmap(cwd + "/models/label_map.pbtxt")
	categories = label_map_util.convert_label_map_to_categories(
	    label_map,
	    max_num_classes=13,
	    use_display_name=False
	)
	self.category_index = label_map_util.create_category_index(categories)

	config = tf.ConfigProto()
	config.gpu_options.allow_growth = True

	detection_graph = tf.Graph()
	with detection_graph.as_default():
	    graph_def = tf.GraphDef()
	    with tf.gfile.GFile(frozen_model_path, 'rb') as fid:
		serialized_graph = fid.read()
		graph_def.ParseFromString(serialized_graph)
		tf.import_graph_def(graph_def, name='')

	    self.sess = tf.Session(graph=detection_graph, config=config)

	    self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
	    self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
	    self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
	    self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
	    self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')
	
	self.graph = detection_graph

	self.light_state_map = {
	    'Red': TrafficLight.RED,
	    'Green': TrafficLight.GREEN,
	    'Yellow': TrafficLight.YELLOW
	}
	self.min_score_thresh = min_score_thresh
	self.processing = False

	rospy.loginfo("light classification model loaded")

    def get_classification(self, image, log_results=False, vis=False):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
	self.processing = True
	
	start_time = time.time()
	light_state = TrafficLight.UNKNOWN
	class_name = ""
	
	image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	image_np = np.asarray(image)
	image_np_expanded = np.expand_dims(image_np, axis=0)
	with self.graph.as_default():
	    (boxes, scores, classes, num) = self.sess.run(
		[
		    self.detection_boxes,
		    self.detection_scores,
		    self.detection_classes,
		    self.num_detections
		],
		feed_dict={self.image_tensor: image_np_expanded}
	    )

	boxes = np.squeeze(boxes)
	classes = np.squeeze(classes).astype(np.int32)
	scores = np.squeeze(scores)

	for i in range(boxes.shape[0]):
	    if scores[i] >= self.min_score_thresh:
		class_name = self.category_index[classes[i]]['name']
		light_state = self.light_state_map.get(class_name, TrafficLight.UNKNOWN)
	
	if log_results:
	    rospy.loginfo(
		"Elapsed Time: %s, light state: %s, %s",
		time.time() - start_time,
		light_state,
		class_name
	    )

	if vis:
	    vis_util.visualize_boxes_and_labels_on_image_array(
		image_np,
		boxes,
		classes,
		scores,
		self.category_index,
		use_normalized_coordinates=True,
		line_thickness=6)
	    plt.figure(figsize=(12,8))
	    plt.imshow(image_np)
	    plt.show()
	
	self.processing = False

        return light_state
