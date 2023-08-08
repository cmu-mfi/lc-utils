Requirements: 

- [easy_handeye](https://github.com/IFL-CAMP/easy_handeye) with [patch](https://github.com/cmu-mfi/lc-utils/blob/main/0001-Changes-for-plc-setup.patch)
- open3d for offline 3d pointcloud visualization (`conda activate plc` on workstation)
  - `lc_wrapper/lcw_save_sweep.py` needs to be run on individual rpi and their `.npy` outputs need to be passed to `viz_3d_single.py`, `viz_3d_registration.py`
