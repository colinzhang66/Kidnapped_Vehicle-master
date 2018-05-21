[//]: # (Image References)
[image1]: ./data/map.png
[image2]: ./data/results.png

# Overview
The Kidnapped project is concerned about using particle filter technique to define the vehicle location and orientation precisely. The particle filter is implemented in C++. 

This following map explains the steps for the localization process:

![alt text][image1]

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

The main purpose is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

![alt text][image2]
