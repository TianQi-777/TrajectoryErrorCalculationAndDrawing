# TrajectoryErrorCalculationAndDrawing
This is a small demo to compare and plot SLAM estimated trajectories and ground-truth trajectories.

The whole demo is tested in **Ubuntu** Platorforms

## Mathematical theory
<div align=center>  
  
![](https://github.com/TianQi-777/TrajectoryErrorCalculationAndDrawing/blob/master/images/formula1.png)
</div>

## Data description
**ground-truth.txt**:ground-truth trajectories data  
**estimate.txt**:estimated trajectories data  

**Data storage form**  
Time  Translation-x  Translation-y  Translation-z  Quaternion-x  Quaternion-y  Quaternion-z  Quaternion-w  

## Additional Prerequisites for this demo
**Pangolin**  
Use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and interface. 
Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

**Sophus**  
Use [Sophus](https://github.com/strasdat/Sophus) for Lie groups commonly used for 2d and 3d geometric problems. 
Dowload and install instructions can be found at: https://github.com/strasdat/Sophus.

## Build and Run
```
cd XX/XX(include estimated.cpp ,estimated.txt ,groundtruth.txt and CMakeLists.txt)  
mkdir build  
cd build  
cmake ..  
make -j2  
./estimated
```

## Result
**Pangolin GUI:** .  
<div align=center>  
  
![](https://github.com/TianQi-777/TrajectoryErrorCalculationAndDrawing/blob/master/images/drawing.png)
</div>





