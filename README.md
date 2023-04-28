# fem_smoother
本项目是对Apollo 6.0参考线平滑（Fem Smooth --QpWithOsqp）部分源码的移植。  
原始输入轨迹为分段不均匀离散点，首先对轨迹进行分段线性插值处理，然后再进行平滑，最后再计算了平滑后轨迹的heading、kappa、dkappa值。  

![Image text](https://github.com/FasonLee/fem_smoother/blob/master/pictures/Compare.svg)


参考链接：  
https://github.com/ApolloAuto/apollo  
https://zhuanlan.zhihu.com/p/371585754  
https://blog.csdn.net/xl_courage/article/details/121569105  

## 安装依赖三方库（Eigen3、Osqp、Osqpeigen、Boost、Protobuf、matplotlib-cpp）:  
Eigen3、Boost、Protobuf
```
sudo apt install libeigen3-dev  
sudo apt install libboost-all-dev  
sudo apt install libprotobuf-dev protobuf-compiler  
```

osqp
```
git clone https://github.com/oxfordcontrol/osqp  
cd osqp  
mkdir build  
cd build  
cmake ..
make  
sudo make install  
```

osqp-eigen
```
git clone https://github.com/robotology/osqp-eigen.git  
cd osqp-eigen  
mkdir build && cd build  
cmake ..  
make  
sudo make install  
```

matplotlib-cpp
```
git clone https://github.com/lava/matplotlib-cpp.git  
cd matplotlib-cpp  
mkdir build && cd build  
cmake ..  
make  
sudo make install  
```

参考链接：  
https://blog.csdn.net/qq_38313901/article/details/119415574  
https://blog.csdn.net/qq_35632833/article/details/116505099  
https://blog.csdn.net/qq_41854911/article/details/119454212  
























