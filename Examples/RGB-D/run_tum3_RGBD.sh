FILE='./associations/fr3_long_office_household.txt'
if test -f "$FILE"; then
    python associate.py /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household/rgb.txt /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household/depth.txt > associations.txt
fi

./rgbd_tum ../../Vocabulary/ORBvoc.txt ./TUM3.yaml /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household ./associations/fr3_long_office_household.txt