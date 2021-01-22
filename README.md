# ORBSLAM2_Dynamic
For rubust localization in dynamic environment.

## 1. References

- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) 
- [DS-SLAM](https://github.com/ivipsourcecode/DS-SLAM)
- [darknet-YOLOv4](https://github.com/AlexeyAB/darknet)

## 2. Prerequisites

- The same as [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).

-  The dynamic library `libdarknet.so` is obtained from [**AlexeyAB/darknet**](https://github.com/AlexeyAB/darknet) and running on CPU. If you use `darknet/libdarknet_GPU.so`  as  `libdarknet.so` for detecting, you need to install cuda and cudnn. 

- The weight files `yolov4.weights` or `yolov4-tiny.weights` should be placed in `darknet/weight/` and you can download them in [**AlexeyAB/darknet**](https://github.com/AlexeyAB/darknet).

## 3. Build and Run

The same as [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2).

## 4. Explain (Chinese)

refer to [动态环境下的ORB-SLAM2_实现鲁棒的定位](https://blog.csdn.net/XindaBlack/article/details/112909593).



