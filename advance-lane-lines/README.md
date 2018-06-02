**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[![Results](https://img.youtube.com/vi/f7brnKQixPs/0.jpg)](https://www.youtube.com/watch?v=f7brnKQixPs)

[//]: # (Image References)

[image1]: ./output_images/distort_undistort_example.png "Undistorted"
[image2]: ./output_images/undistorted2.png "Road Transformed"
[image3a]: ./output_images/treshold1.png "Example to apply threshold"
[image3b]: ./output_images/threshold2.png "Gradient and Channel threshold"
[image3c]: ./output_images/threshold3.png "Stacked and Binary"

[image4]: ./output_images/warped.png "Warp Example"
[image5]: ./output_images/fit_visual.png "Fit Visual"
[image6]: ./output_images/plotted_lane_area.png "Output"
[video1]: ./project_video.mp4 "Video"

### Camera Calibration

#### 1. Find corners of chessboard images and calculate camera matrix and distortion coefficients.

The code for this step is contained in the fourth code cell of the IPython notebook located in "advanced_lane_lines.ipynb".

I start by loading the calibration images, converting those to gray scale format and then passing to the function `find_corners`, I am assuming the chessboard is fixed on the (9, 6) plane dimension; this function will return objpoints and imgpoints, objpoints which is an array of replicated coordinates, imgpoints contains the list of pixel positions of each detected corner over all chessboard images.

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

### Pipeline (single images)

#### 1. Distortion Correction

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image2]

#### 2. Color Gradient and Threshold

I used a combination of gradient applying Sobel in both directions x and y, also I have combined different channels from HLS, HSV and RGB, using channels S,S,L,V and R with different thresholds.
The code for this step is contained in the 12th code cell of the IPython notebook located in "advanced_lane_lines.ipynb".

![alt text][image3a]

![alt text][image3b]

![alt text][image3c]

#### 3. Warped Image

The code for my perspective transform includes a function called `warp()`, which appears in the 14th code cell of the IPython notebook advanced_lane_lines.ipynb.  The `warp()` function takes as inputs an image (`img`), it computes thes source (`src`) and destination (`dst`) points using image shape and harcoded ratios the following manner:

```python
    shape = img.shape

    src = np.float32(
        [
            [.55*shape[1], .64*shape[0]],
            [.45*shape[1], .64*shape[0]],
            [.15*shape[1], shape[0]],
            [.88*shape[1], shape[0]]
        ])

    dst = np.float32(
        [
            [.75*shape[1], 0],
            [.25*shape[1], 0],
            [.25*shape[1], shape[0]],
            [.75*shape[1], shape[0]]
        ])
```

This resulted in the following source and destination points:
```
| Source        | Destination   | 
|:-------------:|:-------------:| 
| 704,    460.7 | 960, 0        | 
| 576,    460.7 | 320, 0        |
| 192,    720   | 320, 720      |
| 1126.4, 720   | 960, 720      |
```

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

#### 4. Fit lane lines

The code for fit a 2nd order polynomial is in the function `find_lane_line()`, which appears in the 17th code cell of the IPython notebook advanced_lane_lines.ipynb.  The `find_lane_line()` function takes as input a binary warped image, computes a histogram to detect the peaks where is suppose to be the lane lines then find the lane lines indexes and pixel positions to fit  two polynomial order 2 for left and right lanes. The image above shows how the polynomial is plotted over the lanes.

![alt text][image5]

#### 5. Radius of curvature

The code for find the radius curvature and car position can be found in the functions `radius()` which appears in the 15th code cell of the IPython notebook advanced_lane_lines.ipynb.

#### 6. Plotted lane line area

I implemented this step in the function `plot_lane_area()`lines which appears in the 16th code cell of the IPython notebook advanced_lane_lines.ipynb.  Here is an example of my result on a test image:

![alt text][image6]

---

### Pipeline (video)

#### This is the final video 

Here's a [link to my video result](https://www.youtube.com/watch?v=f7brnKQixPs)

---

### Discussion

I spent most of the time struggling with channels trying different combinations and check them separately to see how well they contribute to the lane detection S channels from HLS and HSV were good detecting lanes but when road has rework over it the image get pretty noisy. I had to use high threshold on those images but loss lane detection on some tricky parts of the road, finally I make channel combination and to overlap the noisy part I use a `&`(AND) operator instead of `|`(OR) in L channel to overcome some noise.

After applying channel and gradient thresholding and get good result next step was work in warp lane lines, it was more quicker task by identifying a polygon with four points using the image sizes and ratios that I tuned manually to fit the lines assuming full plain road. Then worked on pipeline to detect lane pixels and fit the second order polynomial on it in order to determine the lane boundary. Finally warp back the image a plot and add the curvature radius and the vehicle position

I used the Line class to store the state of the road regarding radius, position and the flag detected to avoid window searching of lane lines in next frame.

I've tried the challenge video it does not work well, I think my solution over fit the video project, I would spent more time on channel combination and gradient thresholds looking for generalize lane finding over other videos. Also trying new techniques like using neural networks to predict lane lines position would interesting to explore.

Finally the polygon used to warp the image an detect lanes it is quite fixed, I would like to explore what other techniques to use when you have tricky roads with pronounce curves.

