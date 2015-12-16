# Proper

Vision guided 3D Direct Laser Repair of Complex Metal Parts.

## 3D Scan

![3D scan screenshot](./proper/media/scan.png)

To record a bag file with the scanning information:

```
roslaunch proper_workcell proper_bagrecord.launch filename:=scan.bag
```

To play a bag file with the scanning information:

```
roslaunch proper_workcell proper_bagplay.launch filename:=scan.bag
```

To convert a bag file with the scanning information to a point cloud in xyz format:

```
roscore
rosrun proper_cloud sub_cloud.py
roslaunch proper_workcell proper_bagplay.launch filename:=scan.bag
```

## Acknowledgement

This work has been supported by the European Commission through the research project
"Laser equipment ASsessment for High impAct innovation in the manufactuRing European
industry (LASHARE)", FP7-2013-NMP-ICT-FOF grant agreement Nº609046.
