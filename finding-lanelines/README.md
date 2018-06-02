#**Finding Lane Lines on the Road**

## Intro
In self-driving car detecting lane lines it is necessary to guide cars in the roads
this project shows several techniques needed to achieve this important task. The
goal it is to find both left and right lane lines by plotting colored lines over its
respective lane lines pixels in the image.


---

**Finding Lane Lines on the Road**

The goals of this project are the following:

* Process input image and detect lines
* Filter detected lines to match the lane lines
* Drawn the left and right corresponding lane lines into the image
* Make a pipeline that finds lane lines on the road
* Apply this pipeline to  the test set images and videos provided
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./examples/grayscale.jpg "Grayscale"

---

### Reflection

### 1. Description
  The following steps describe how the pipeline was built.
  
  1. Detect Image Edges
    Using canny technique to determine edges over all image, it requires the input
    image to be of one channel (gray scale), then a gaussian function with some kernel
    configuration should be apply to remove some noise and finally plug the output of this
    to canny function.

  2. Detect lines from Hough Space
    This step detect lines from a given image, to facilitate it the input image
    already one with only edges and in addition it receive a mask as region of
    interest to only focus serching lines in given region. The output it is a collection
    of lines(two points) of different slopes

  3. Drawn the lines which overlap on lane lines.
    
    The `draw_lines` function was exended in order to improve the drawing of the lane lines,
    the hough lines of given image were filtered to separate right lane lines from
    left lane lines. Also to filter out noisy lines that does not correspond to the lane lanes.
    Hough lines with positive slope are right ones and the ones with negative slope are the left ones.
    Lines wich probably are the ones that overlap the lane lines should have slopes between this range
    (0.5..0.9) and (-0.5..-0.9) other than those ranges are considered noise.

    Having the curated list of lines the next is join all of these by averaging slope and center points.
    then using this slope(m) and center points (x1, y1) extrapolate the line with the help of this
    formula (y - y1) = m(x - y1); in turn out thif will lead to only two lines for left and right lanes. I've
    created a helpers function `extrapolate_line` and `filter_lines` to keep code of `draw_lines` simple and
    clean as possible.
    
    Sudden movements from the lines between frames were tackle by averaging over all slopes, and centers
    of all frames, it was necesary to soften the drawn between frames. Also I conrideder only average over the
    last 100 paramaters which behave well. I did experiments with 10 but lines was shaking a little, then tried
    1000 and lines some times get out of the lane, but looks like 100 it is a good value for keep the k last 
    parameters over last frames.
    
    As an improvement to understand what is happening in the image I added the averaged slope and the current center
    for every frame.


  After work over these 3 steps, now there is a foundations that can be use to create the functions
  `get_canny_edges`, `detect_lines` and `drawn_lane_lines` which wil join all provided helper functions
  to detect and draw the lines over the test images as shown above:

  ![alt text][image1]
  
  ![alt text][image3]

  [//]: # (Image References)
  [image1]: ./examples/image_0.jpg "solidWhiteCurve"
  [image2]: ./examples/image_1.jpg "solidWhiteRight"
  [image3]: ./examples/image_2.jpg "solidYellowCurve"
  [image4]: ./examples/image_3.jpg "solidYellowCurve2"
  [image5]: ./examples/image_4.jpg "solidYellowLeft"
  [image6]: ./examples/image_5.jpg "whiteCarLaneSwitch"


   I've tried to play with `region_of_interest` and `HSV` format to improve the work above, you can find this
   code at the very bottom of the notebook.

### 2. Identify potential shortcomings with your current pipeline
I noted that the slope of different lines vary a lot it makes some times the line to 
dont' stick exactly in the lane line in some curves.

Other shortcomming could be if the road has shadows and other residual material that is near
the lane lines averaging the slopes and center may don't match at all on the lane lines.



### 3. Suggest possible improvements to your pipeline
I've tried use the function cv2.inRange for filter out colors except yellow and white this way
identifying lines coulbe much more easy for canny and hough spaces, but it doesn't show to much
improvement since tunning and guessing the color ranges for this seem tricky. An posible improvement
should be try to parametrize well this function go get better results.

An other improvement could be figure it out how to adapt automaticaly the bottom and top position of
the lines to match the image, for example for challenge video the lines do well but one line is more
taller than the other. would be nice have a way to get the lines in same top or bottom level.



