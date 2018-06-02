**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./examples/car_not_car.png
[image2]: ./examples/HOG_example.png
[image3]: ./examples/sliding_windows.png
[image4]: ./examples/sliding_window.png
[image5]: ./examples/bboxes_and_heat.png
[image6]: ./examples/labels_map.png
[image7]: ./examples/output_bboxes.png
[video1]: ./project_video.mp4

---

### Histogram of Oriented Gradients (HOG)

#### 1. HOG features from the training images.

The code for this step is contained in the fourth code cell of the IPython notebook in the function get_hog_features.  

I started by reading in all the `vehicle` and `non-vehicle` images.  Here is an example of one of each of the `vehicle` and `non-vehicle` classes:

![alt text][image1]

I then explored different color spaces and different `skimage.hog()` parameters (`orientations`, `pixels_per_cell`, and `cells_per_block`).  I grabbed random images from each of the two classes and displayed them to get a feel for what the `skimage.hog()` output looks like.

Here is an example using the `YUV` color space and HOG parameters of `orientations=11`, `pixels_per_cell=(16, 16)` and `cells_per_block=(2, 2)`:


![alt text][image2]

#### 2. HOG parameters.

I tried various combinations of parameters cells per block 2, pixels per cell 8 and 16 orient 6, 9, 11 , 12.
8 pixels per cell give a very good clear shape of cars but using 16 pixels per cell give good shape and less features that will be fasted to train a predict. Regarding orientation 11 has good fit for horizontal lines and performs well in training, compare with other orientations 9 and 12 it didn't show too much differences.

#### 3. Classifier
The code for this step is contained in the sixth code cell of the IPython notebook.  

I trained a SVM using using rbf, I tunned it using cross validation grid search, trying with linear, poly, sigmoid and rbf. Below there is an example how I tunned parameters and plot results
```
#tuned_parameters = [{'kernel': ['sigmoid'], 'gamma': [ 0.0001, 0.00003], 'C': [ 0.5, 1 ]}]
#clf = GridSearchCV(SVC(verbose=True), tuned_parameters, cv=4)
.
.
.
#print("Best Parameters")
#print(clf.best_params_)
```

I use a training set of car and not cars roughly 14.000 images, I perform gridsearch over 500 and 1000 examples
Once I get the best parameters I've trained again the SVM with 10000 images and final model was trained with 14000 images.

The data was shuffle and then split in training and test sets, parameter to train were `gamma=0.0001, C=0.5, kernel="rbf"`, time to train was 69.54 and test accuracy score was `0.9939`

Images were converted to from RGB to YUV space color. I used Hog, histogram bins and image binning as features all of it extracted and concatenated in one single vector per car o not car in this function `extract_features()`. Data was scaled and centered using scikit learn `StandardScaler` function.

Using the helper functions provided by Udacity I extracted out features for histogram I used 32 bins, for image resize use (32, 32) and for Hog these parameters orient 11, pix_per_cell 16, cell_per_block 2.

### Sliding Window Search

#### 1. Sliding window search
The code for this step is contained in the eighth code cell of the IPython notebook in the function
detect_windows.

I used three sets of boxes:
  - size (64, 64), with 50% overlap in both x and y
  - size (100, 100), with 60% overlap in x and  80% overlap in y
  - size (200, 200), with 60% overlap in both x and y

In total 444 bounding boxes in the half of the image to avoid false positives in the sky, Also the start x,y and stop x,y was tunned to control number of boxes.

The images below shows an example of the boxes used to search cars.

![alt text][image3]

#### 2. Detection examples

Initially I used YCrCb, HLS and HSV, with 8 pixel per cell for HOG features, training was taking 2 minutes and in the prediction I was getting several false positives, then I searched on three scales using YUV 3-channel HOG features plus spatially binned color and histograms of color in the feature vector, 16 pixel per cell for HOG features and head map counting which provided a nice result.  Here are some example images:

![alt text][image4]
---

### Video Implementation

#### 1. Video Result

Here's a [link to my video result](./project_out.mp4)


#### 2. False positives.
The code for this step is contained in the seventh code cell of the IPython notebook in the function
heat_windows.


I did not have false positive issues however I follow suggestion on lectures to add the heatmap over images.
I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  

Here's an example result showing the heatmap from a series of frames of video, the result of `scipy.ndimage.measurements.label()` and the bounding boxes then overlaid on the last frame of video:

### Here are six frames and their corresponding heatmaps:

![alt text][image5]

### Here is the output of `scipy.ndimage.measurements.label()` on the integrated heatmap from all six frames:
![alt text][image6]

### Here the resulting bounding boxes are drawn onto the last frame in the series:
![alt text][image7]


---

###Discussion

Initially had lot of problems to get the classifier works well with few problems:

  * The data was being loaded in the order in which the function glob.glob load files, I was selecting 1000 images but in that order, despite high accuracy in test it fails to recognize cars in the test images. I fixed by shuffle the file list before select the number of examples I wanted to use on training.

  * Tunning SVM with grid search was painful cause it took a lot of time. I managed to work with small data set and train with no to large options of hyper parameters to come with some approximation quickly.

  * Since the training images of car are very fit, it do not have more background and just the shape of car in the image, the classifier was not performing well with windows that cover some background like trees and mountains. I fix it by using different windows scales.

This pipe line will fail in other videos with different road, I hard coded the windows slides, so maybe it is not robust in other scenarios. Also it took 2 minutes to process the 2 second test video and 1 hour and 30 minutes the project video with has 1261 frames.

I have tried to apply the Hog feature sub-sampling extraction, but could make it work at this time, I was trying to figure it out how to use different windows sizes in the grid to extract features of different locations. I would like to go further by using this technique an see how fast or nearly real time detection it can do.

Other improvement I would love to do it is change SVM classifier by a ConvNet to take advance of the statistical invariant feature of these nets.


