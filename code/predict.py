# import packages
import argparse
#from arguments import Args
import cv2
import imutils
from imutils import paths
import numpy as np
import os
import random
from skimage import transform
from skimage import exposure
from skimage import io
from tensorflow.keras.models import load_model
#from train import Train_Net

class Predict_Net:

	
	def main_predict_net(self):

		#self.tr = Train_Net()
		#arg = Args()
		#self.args = arg.parse_arguments()

		self.load_net()
		self.prediction_process()
		#self.publish_label(label)

		return self.label


	def load_net(self):

		# load the trained model
		print("[INFO] loading model...")
		self.model = load_model("../output/neural_net.model")

	def get_sign_names(self):
		
		# load sign names
		#CHANGE: _rl = turn right/left only, _all = all signs
		#sign_names = open("../sign_names_all.csv").read().strip().split("\n")[1:]
		sign_names = open("../sign_names_rl.csv").read().strip().split("\n")[1:]
		
		sign_names = [s.split(";")[1] for s in sign_names]

		return sign_names


	def prediction_process(self):

		# grab the paths to the input images, shuffle them, and grab a sample
		print("[INFO] predicting...")

		sign_names = self.get_sign_names()

		paths_to_image = list(paths.list_images("../gtsrb_rl/Test"))
		random.shuffle(paths_to_image)

		# choose only 30 images
		paths_to_image = paths_to_image[:1]

		# loop over the image paths
		for (i, path_to_image) in enumerate(paths_to_image):
			
			# resize images and perform CLAHE
			image = io.imread(path_to_image)
			image = transform.resize(image, (32, 32))
			image = exposure.equalize_adapthist(image, clip_limit=0.1)

			# preprocess the image by scaling it to the range [0, 1]
			image = image.astype("float32") / 255.0
			image = np.expand_dims(image, axis=0)

			# make predictions using the traffic sign recognizer CNN
			predictions = self.model.predict(image)
			j = predictions.argmax(axis=1)[0]
			self.label = sign_names[j]

			# load the image using OpenCV, resize it, and draw the label
			image = cv2.imread(path_to_image)
			image = imutils.resize(image, width=128)
			cv2.putText(image, self.label, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

			# save the image to disk
			p = os.path.sep.join(["../predictions_rl", "{}.png".format(i)])

			cv2.imwrite(p, image)

			#print(self.label)
			return self.label

'''
	def publish_label(self, label):

		pub = rospy.Publisher('ros_label', String, queue_size=10)
		rospy.init_node("ros_label", anonymous=True)
		rate = rospy.Rate(1)

		while not rospy.is_shutdown():

			pub.publish(label)
			rospy.loginfo("Please turn %s", label)
			rate.sleep()

'''

if __name__ == "__main__":
    	
	p = Predict_Net()
	p.main_predict_net()

