## Project: Search and Sample Return
d
**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
I really don't know how to answer it. All I can say is that I did this step.

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
In the 15th code cell, `process_image()` is filled and other functions are defined above. The output of my own data is uploaded. 

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.
In the `perception_step()`, I used three functions with different conditions/ thresholds to identify rock, road and hills. I only pass central view to decision_step to prevent rover from driving in S curve.
In the `decision_step()`, a plan is made combining the known rock locations and nodes I defined in `drive_rover.py`. The nodes are defined to prevent rover to drive back and forth in same route. Then the rover will follow the plan to do the mapping and pick up all rocks in the way. The rest code is just to cooperate with the plan.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  
It will fail when a rock is hide behind a big rock.
Otherwise, it will map about 98% with fidelity of only 77%. The map is definately mapped nearly 100% because I have already define nodes to guide the rover. The terrible fidelity is a result of frequent stop and the view is shaken. With pitch other than 0, the mapping is not accurate.
I will make it stop smoothly rather than just use `brake = 10` and only update map if pitch is small or its speed is not too small.

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**
Resolution is 1280 * 720
Quality is fastest
FPS is about 30

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  

First, I tried to make a wall crawller which keeps the wall to its right and only turn left when it comes to a dead end. But it will drive in S curve and sometimes just drive to the left wall which scews this idea.

Then, I tried to identify the wall then make the rover drive close to it. But it usually stuck in the wall because it is too close and it cannot pick up rocks because it has stuck in the wall.

Finally, I used nodes to define the basic route and add rocks to the routh with some opperations which are not intelligent at all. It works fine, but this method depends on human to define nodes to guide the rover. 

