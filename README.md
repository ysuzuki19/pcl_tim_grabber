# pcl_tim_grabber

```
The grabber implemented this repository was merged on pcl (#4429).

This repository is only example of running grabber and visualize data.
```

grabber class for SICK sensor named Tim.

# demo

![tim_grabber_demo](/tim_grabber_demo.gif)

# Dependencies

Please build and install with master branch of [pcl](https://github.com/PointCloudLibrary/pcl).

## Linux

[here](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html#experimental)

If you run with other platform, please go to the page from [here](https://github.com/PointCloudLibrary/pcl#compiling)

# Connect Tim5xx

Connect Tim5xx and set ip `192.168.0.XXX` (same LAN group)

# Run example

```
$ git clone https://github.com/ysuzuki19/pcl_sick_grabber
$ makdir build
$ cd build
$ cmake ..
$ make
$ ./visualize_tim
```
