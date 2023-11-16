# Range-aided Localization
This repository contains code for range-aided localization. The code supports two different modes of sensor fusion. More details on the algorithm will be made available shortly.

## Citation

If you use this repository in your research, please use the following citation:

```
@misc{goudar2023rangevisualinertial,
      title={Range-Visual-Inertial Sensor Fusion for Micro Aerial Vehicle Localization and Navigation}, 
      author={Abhishek Goudar and Wenda Zhao and Angela P. Schoellig},
      year={2023},
      eprint={2311.09056},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Note
Not all features of the algorithm have been merged. We are in the process of updating the code base. 

## Installation
For installing necessary dependencies and building instructions please see [installation](https://github.com/utiasDSL/ra_lan/tree/main#dependencies).

## Testing with LSY Range-Aided Localization dataset

Please ensure you have followed the necessary installation steps before proceeding.

The functionality of the code can be tested using a rosbag from the [LSY Range-Aided Localization](https://utiasdsl.github.io/utias_ra_loc) dataset. 

1. Download the [UTIAS_vicon_12122022](https://utiasdsl.github.io/utias_ra_loc/03_UTIAS_vicon_1212022.html#data-files) dataset.

2. Unzip the dataset.

3. Run the launch file
```
source ~/catkin_ws/devel/setup.bash
roslaunch ra_sam default.launch --screen
```

4. Play the rosbag from dataset

```
source ~/catkin_ws/deve/setup.bash
rosbag play <path-to-dataset>/trial1/sensor_data.bag
```

5. If everything works correctly, you should see the following visualization in Rviz:

<p align="center">
    <img src="doc/assets/ra_sam_trial1.gif" alt="drawing" width="600"/>
</p>


## Testing with different datasets

To test with different rosbags from LSY Range-Aided Localization dataset, edit the `default.launch` file insert the path of `robot_config.yaml` for the corresponding dataset in the `robot_config` field (highlighted below) and repeat the steps from the previous section.

<p align="center">
    <img src="doc/assets/edit_launch_file.png" alt="drawing" width="800"/>
</p>

## Known issues

1. Odometry preintegration's covariance is approximated right now. Covariance prediction needs to be merged from the corresponding branch.


