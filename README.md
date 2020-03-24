[//]: # (Image References)

[image1]: ./img/algo.png "Algorithm" 
[image2]: ./img/yaw_const.png "Bicycle Model Yaw Const" 
[image3]: ./img/yaw_var.png "Bicycle Model Yaw Var" 
[image4]: ./img/trans.png "Homogeneous Transformation" 
[image5]: ./img/multi_var.png "Multi variate Gaussian Distribution"

# Kidnapped Vehicle Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This repository contains all the code for the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

## Overview

Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

The sparse localization problem stated above is solved by implementing a **2 dimensional Particle Filter in C++**. 

The particle filter is given a map in the form of a list of `(x position, y position, landmark id)` and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.


## Installation & Run

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems : `install_linux.sh` and `install_mac.sh`. 

For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

    mkdir build
    cd build
    cmake ..
    make
    ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. `./clean.sh`
2. `./build.sh`
3. `./run.sh`

#### Notes

Details about simulation protocols are given in [this appendix file](./appendix_sim_protocol.md)

## Algorithm

All the code for the particle filter is contained in class `ParticleFilter` define in file [particle_filter.h](./src/particle_filter.h) and implemented in file [particle_filter.cpp](./src/particle_filter.cpp).

Some helper functions are contained in file [particle_utils.h](./src/particle_utils.h). Here there are contained generic functions used by particle filter algorithm but that can be easily used in other contexts.

The particle filter algorithm is presented in the image below.

![alt text][image1]
*From Udacity Localization Lesson. Image Credits to Udacity*


Its implementation is mainly divided into 5 steps.


#### 1. Belief Initialization:

It is needed to initialize our particles. It is then needed to make an initial estimate using GPS input. 

As with all sensor based operations, this step is impacted by noise.

Particles are so initialized by sampling a Gaussian distribution, taking into account Gaussian sensor noise around the initial GPS position and heading estimates. This noises are specified via the array variable `sigma_pos` defined at **line 36** of `particle_filter.h`.

The initialization code is contained in function `init` at **lines 26-64**  of  `particle_filter.cpp`.

#### 2. Prediction: Bicycle Model

Now that particles are initialized it’s time to predict the vehicle’s position. 

Here there are used the equations for updating vehicle position modeled with **Bicycle Model**

Yaw Rate Constant | Yaw Rate Variable
--|--
![alt text][image2] | ![alt text][image3]

*From Udacity Localization Lesson. Image Credits to Udacity*

In addition to this state evolution, Gaussian sensor noise is also taken into account via the array variable `sigma_pos` defined at **line 36** of `particle_filter.h`. Sensor noise is here modeled with zero mean and standard deviation specified by `sigma_pos`.

The code for this step is contained in function `runBicycleModel` defined at **lines 29-55** of `particle_utils.h`

#### 3. Transform

At this point, all particles are updated to their predicted position. Now it is needed to map sensor observations to map landmark around each particle.

Sensor observations are given in vehicle's coordinate system while map landmarks are given in map global coordinate system.

In order to map them a transformation is needed. Here it is used a **Homogeneous Transformation** of sensor observation to map coordinate system.

The transformation is achieved via the equations:

![alt text][image4]
*From Udacity Localization Lesson. Image Credits to Udacity*

where

* `(xc, yc)` is the observation coordinate, in car coordinate system
* `(xp, yp)` is the particle coordinates, in map coordinate system
* `theta` is the rotation angle of the transformation
* `(xm, ym)` is the observation coordinate, in map coordinate system for that particle

The code for this step is contained in function `getTransformation` defined at **lines 99-112** of `particle_utils.h`

#### 4. Data Association: Nearest Neighbor

Once all coordinate are referenced to the same coordinate system, it is needed to associate each sensor observations to map landmark

Here the **nearest neighbor** policy is chosen to match observation and map landmarks.

For each observation there is a search for nearest map landmark and its id is set to observation id in order to store this match.

The code for this step is contained in function `dataAssociation` defined at **lines 104-141** of `particle_filter.cpp`


#### 5. Weights Update

New weight value for each particle is computed using the multi-variate Gaussian probability density function .

![alt text][image5]

*From Udacity Localization Lesson. Image Credits to Udacity*

This function tells us how likely a set of landmark measurements is given our predicted state of the car and the assumption that the sensors have Gaussian noise.

We also assume that each landmark measurement is independent, so we take the product of the likelihood over all measurements.

In the formula:

* `x_i` is the i-th landmark measurement for one particular particle
* `mu_i` is the predicted measurement for the map landmark corresponding to the i-th measurement
* `m` is the total number of measurements for one particle
* `sigma` is the covariance of the measurement. It is a symmetric quadratic matrix representing uncertainty over x, y on the diagonal and correlated uncertainties x over y and y over x in other cells

The code for this step is contained in function `updateWeights` of `particle_filter.cpp`. In particular weight updating is implemented at **lines 212-246**. 

For each particle the code block specified above uses the function `multiv_prob` defined at **lines 69-86** of `particle_utils.h`

## Results

#### Success Criteria

Success is verified by the simulator and the result is presented on the simulator window with the message:

    "Success! Your particle filter passed!"

The metrics the grading code is looking for are:

1. **Accuracy**: the particle filter should localize vehicle position and yaw to within the simulator specified parameters.

2. **Performance**: the particle filter should complete execution within the time of 100 seconds.

#### Performance Results

In the table below there are shown the result with different number of particles (5, 100, 200, 400, 1000).

Different tests have been led in order to evaluate the scalability robustness and the performance improvement with respect to the number of particles selected.

Num Particles | X error | Y Error | Yaw Error | End System Time
--- | --- | --- | --- | ---
5 | .208 | .191 | .007 | 49s |
**100** | **.114** | **.108** | **.004** | **69s** |
400 | .110 | .101 | .004 | 102s |
1000 | .109 | .103 | .004 | >100s |

Some considerations can be made by the result table above:

* Even with just 5 particles the filter is able to passing the simulator grading process. Its error in location is about 20 cm for both x and y coordinate.
* With 100 particles, the error reduces to 10 cm for both coordinates. This results is definitely better for localization purpose since a self driving car must be capable of localize itself with a precision of at least 10 cm.
* 400 particles and 1000 particles cases show that increasing the number of particles affect a lot the computation time and lead to non-significant improvement in performance.

Since the consideration made on particles number the released version has **100 particles**.

## Video Result

In the video below there are shown Particle Filter performance using 100 particles.

The blue lines are debug information activated via preprocessor directive `#define DEBUG_INFO` located at **line 20** of `particle_filter.h` file. 

The green laser sensors from the car nearly overlap the blue laser sensors from the particle, this means that the particle transition calculations were done correctly.

:[![final](https://img.youtube.com/vi/OyGvqU18L6Y/0.jpg)](https://www.youtube.com/watch?v=OyGvqU18L6Y) 
