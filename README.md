# TrackAlignmentWith_ICP
This is a small demo to compare and plot SLAM estimated trajectories and ground-truth trajectories with ICP.

The whole project is tested in **Ubuntu** Platorforms

## Mathematical derivation
<div align=center>  
  
![](https://github.com/TianQi-777/TrackAlignmentWith_ICP/blob/master/images/formula1.png)
</div>

## Data description
**ground-truth.txt**:ground-truth trajectories data  
**estimate.txt**:estimated trajectories data  

**Data storage form**  
Time  Translation-x  Translation-y  Translation-z  Quaternion-x  Quaternion-y  Quaternion-z  Quaternion-w  

## Additional Prerequisites for this project
**Pangolin**  
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and interface. 
Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

