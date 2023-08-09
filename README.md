## Requirements: 

- [easy_handeye](https://github.com/IFL-CAMP/easy_handeye) with [patch](https://github.com/cmu-mfi/lc-utils/blob/main/0001-Changes-for-plc-setup.patch)
- open3d for offline 3d pointcloud visualization (`conda activate plc` on workstation)
  - `lc_wrapper/lcw_save_sweep.py` needs to be run on individual rpi and their `.npy` outputs need to be passed to `viz_3d_single.py`, `viz_3d_registration.py`
 

## Testbed demo
*as prepared for 07.31.2023*

All four robots and two light curtain sensors must be powered on and connected to the network. Make sure the lc-rpi devices are in sync (`sudo ntpdate 192.168.1.1`)

### Workstation (mfi-twin)
`mfi@192.168.1.1` <!--(pwd: lego)-->

- `roslaunch lego_moveit EAB_demo.launch`
  - This will start all four robots, their respective moveit rviz windows, and their demo motion scripts.
  - If you don't want to start the demo motion scripts, you can start the robots with `roslaunch lego_moveit lego_moveit_all.launch` instead.

- `roslaunch lc_utils setup_demo.launch`
  - Starts node to publish joint positions of all robots needed for designing the light curtains. 
  - Publishes transforms between cameras, robots, and lasers.
  - Starts the demo rviz for the light curtain visualizations. 

### redversion (lc-rpi)
`ilim@192.168.1.5` <!--(pwd: ilimlab)-->
- `roslaunch plc_robot yaskawa_hull_1_2.launch`
   - Starts nodes for sending out safety curtains and monitoring them for intrusions.
   - Starts node for visualizing safety curtains on helper camera images.
   - Starts eye-safety nodes (untested).

### redversion (lc-rpi)
`ilim@192.168.1.4` <!--(pwd: ilimlab)-->
- `roslaunch plc_robot yaskawa_hull_1_2.launch`
   - Starts nodes for sending out safety curtains and monitoring them for intrusions.
   - Starts node for visualizing safety curtains on helper camera images.
   - Starts eye-safety nodes (untested).
