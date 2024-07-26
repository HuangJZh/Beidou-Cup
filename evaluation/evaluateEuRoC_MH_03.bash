#! /bin/bash

# copy 'eval_cfg.yaml' and 'stamped_groundtruth.txt' from previous successful evaluation to the test result directory
RESULT_PATH="/home/xin/fpv_2020/compare/FAIM_results/euroc_mono_stereo/laptop/vio_stereo/laptop_vio_stereo_MH_03-FAIM-20231002"
PREVIOUS_RESULT_PATH="/home/xin/fpv_2020/compare/FAIM_results/euroc_mono_stereo/laptop/vio_stereo/laptop_vio_stereo_MH_03-FAIM-pretest/"
rm $RESULT_PATH -rf
mkdir $RESULT_PATH
cp $PREVIOUS_RESULT_PATH/eval_cfg.yaml $RESULT_PATH
cp $PREVIOUS_RESULT_PATH/stamped_groundtruth.txt $RESULT_PATH


# convert 'traj.csv'  to 'groundtruth.txt', which is renamed as 'stamped_traj_estimate.txt' and cut & pasted to the test result directory
FAIM_OUTPUT_PATH="/home/xin/FAIM/output/EuRoC_MAV/machine_hall/MH_03"
SCRIPT_PATH="/home/xin/fpv_2020/compare/scripts"
python2 $SCRIPT_PATH/dataset_tools/asl_groundtruth_to_pose.py \
         $FAIM_OUTPUT_PATH/traj_vio.csv
mv $FAIM_OUTPUT_PATH/groundtruth.txt $FAIM_OUTPUT_PATH/stamped_traj_estimate.txt
mv $FAIM_OUTPUT_PATH/stamped_traj_estimate.txt $RESULT_PATH

# run the evaluation
python2 $SCRIPT_PATH/analyze_trajectory_single.py \
	     $RESULT_PATH

# remove the output file
rm $FAIM_OUTPUT_PATH/traj_vio.csv
