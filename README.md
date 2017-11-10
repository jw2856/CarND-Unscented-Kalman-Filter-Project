# Unscented Kalman Filter Project

This program implements an Unscented Kalman filter as part of Term 2 of Udacity's Self-Driving Car Engineer Nanodegree Program.

## Installation

### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

### Instructions

1. Clone this repo.

```
$ git clone git@github.com:jw2856/CarND-Unscented-Kalman-Filter-Project.git
```

2. If a build directory doesn't exist, make a build directory.

```
$ mkdir build && cd build
```

3. Compile:

```
$ cmake .. && make
```
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`

4. Run it

```
$ ./UnscentedKF 
```

## Usage

Once you've built and run this program, you'll need to hook it up to Udacity's Term 2 simulator to see the results. You can find that [here](https://github.com/udacity/self-driving-car-sim/releases). When the simulator connects, you can run the simulator and see the results.