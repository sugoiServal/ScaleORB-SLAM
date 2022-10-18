#!/bin/sh
# cd "/mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/Examples/RGB-D"

# FILE='./associations/fr1_360.txt'
# python associate.py /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_360/rgb.txt /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_360/depth.txt > $FILE
# FILE='./associations/fr1_desk.txt'
# python associate.py /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_desk/rgb.txt /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_desk/depth.txt > $FILE
# FILE='./associations/fr1_room.txt'
# python associate.py /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_room/rgb.txt /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_room/depth.txt > $FILE
# FILE='./associations/fr3_sitting_halfsphere.txt'
# python associate.py /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_sitting_halfsphere/rgb.txt /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_sitting_halfsphere/depth.txt > $FILE




# cd "/mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/debug"
# pwd
# fr1_360  # fr1_360  # fr1_360  # fr1_360  # fr1_360  # fr1_360  # fr1_360  # fr1_360  # fr1_360  # fr1_360  
# for status in $(seq 1 1); do  
#     ./mono_tum ../Vocabulary/ORBvoc.txt \
#         ../Examples/Monocular/TUM1.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg1_360 \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_360
#     _pid = $!
#     wait $_pid
# done


# for status in $(seq 1 1); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM1.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg1_360 \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_360 \
#         ../Examples/RGB-D/associations/fr1_360.txt;
#     _pid = $!
#     wait $_pid
# done

# fr1_desk  # fr1_desk  # fr1_desk  # fr1_desk  # fr1_desk  # fr1_desk  # fr1_desk  # fr1_desk  # fr1_desk  
# for status in $(seq 1 2); do  
#     ./mono_tum ../Vocabulary/ORBvoc.txt \
#         ../Examples/Monocular/TUM1.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg1_desk \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_desk
#     _pid = $!
#     wait $_pid
# done


# for status in $(seq 1 49); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM1.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg1_desk \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_desk\
#         ../Examples/RGB-D/associations/fr1_desk.txt;
#     _pid = $!
#     wait $_pid
# done      


# for status in $(seq 1 5); do  
#     ./mono_tum ../Vocabulary/ORBvoc.txt \
#         ../Examples/Monocular/TUM3.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg3_teddy \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_teddy
# done


# for status in $(seq 1 25); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM3.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg3_teddy \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_teddy \
#         ../Examples/RGB-D/associations/fr3_teddy.txt;

# done

# freiburg1_room  # freiburg1_room  # freiburg1_room  # freiburg1_room  # freiburg1_room  # freiburg1_room  
# for status in $(seq 1 15); do  
#     ./mono_tum ../Vocabulary/ORBvoc.txt \
#         ../Examples/Monocular/TUM1.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg1_room \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_room

# done


# for status in $(seq 1 25); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM1.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg1_room \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_room \
#         ../Examples/RGB-D/associations/fr1_room.txt;
# done

# # freiburg3_sitting_halfsphere  # freiburg3_sitting_halfsphere  # freiburg3_sitting_halfsphere  # freiburg3_sitting_halfsphere  
# for status in $(seq 1 1); do  
#     ./mono_tum ../Vocabulary/ORBvoc.txt \
#         ../Examples/Monocular/TUM3.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg3_sitting_halfsphere \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_sitting_halfsphere
#     _pid = $!
#     wait $_pid
# done


# for status in $(seq 1 1); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM3.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg3_sitting_halfsphere \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_sitting_halfsphere \
#         ../Examples/RGB-D/associations/fr3_sitting_halfsphere.txt;
#     _pid = $!
#     wait $_pid
# done

# fr1_xyz
# cd "/mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/Examples/RGB-D"
# FILE='./associations/fr1_xyz.txt'
# python associate.py /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_xyz/rgb.txt /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_xyz/depth.txt > $FILE
# cd "/mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/debug"

# for status in $(seq 1 1); do  
#     ./mono_tum ../Vocabulary/ORBvoc.txt \
#         ../Examples/Monocular/TUM1.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg1_xyz \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_xyz
#     _pid = $!
#     wait $_pid
# done


# for status in $(seq 1 1); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM1.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg1_xyz \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg1_xyz\
#         ../Examples/RGB-D/associations/fr1_xyz.txt;
#     _pid = $!
#     wait $_pid
# done

# fr2_xyz  fr2_xyz  fr2_xyz  fr2_xyz  fr2_xyz  fr2_xyz  fr2_xyz  
# cd "/mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/Examples/RGB-D"
# FILE='./associations/fr2_xyz.txt'
# python associate.py /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg2_xyz/rgb.txt /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg2_xyz/depth.txt > $FILE
# cd "/mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/debug"

# for status in $(seq 1 14); do  
#     ./mono_tum ../Vocabulary/ORBvoc.txt \
#         ../Examples/Monocular/TUM2.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg2_xyz \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg2_xyz
#     _pid = $!
#     wait $_pid
# done


# for status in $(seq 1 49); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM1.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg2_xyz \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg2_xyz\
#         ../Examples/RGB-D/associations/fr2_xyz.txt;
#     _pid = $!
#     wait $_pid
# done

# fr3_lon_office
# for status in $(seq 1 5); do  
#     ./mono_tum ../Vocabulary/ORBvoc.txt \
#         ../Examples/Monocular/TUM3.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg3_long_office \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household
# done


# for status in $(seq 1 49); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM1.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg2_xyz \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg2_xyz\
#         ../Examples/RGB-D/associations/fr2_xyz.txt;
#     _pid = $!
#     wait $_pid
# done

#fr3_lon_office
cd /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning
# mask_thres, conf thres
CUR_D='0608'
mkdir $CUR_D && cd $CUR_D && mkdir Scale && cd Scale && mkdir Builder && mkdir Mono
cd .. && mkdir Trajectory && cd Trajectory && mkdir Builder && mkdir Mono
cd /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/debug

for status in $(seq 1 5); do  
    ./mono_tum ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/TUM3.yaml \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning/$CUR_D \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
        /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household \
        0.6 0.8;
done

# for status in $(seq 1 30); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM3.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning/$CUR_D \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household\
#         ../Examples/RGB-D/associations/fr3_long_office_household.txt \
#         0.6 0.8;
# done

cd /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning
# mask_thres, conf thres
CUR_D='0607'
mkdir $CUR_D && cd $CUR_D && mkdir Scale && cd Scale && mkdir Builder && mkdir Mono
cd .. && mkdir Trajectory && cd Trajectory && mkdir Builder && mkdir Mono
cd /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/debug

for status in $(seq 1 5); do  
    ./mono_tum ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/TUM3.yaml \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning/$CUR_D \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
        /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household \
        0.6 0.7;
done

# for status in $(seq 1 20); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM3.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning/$CUR_D \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household\
#         ../Examples/RGB-D/associations/fr3_long_office_household.txt \
#         0.6 0.7;
# done




cd /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning
# mask_thres, conf thres
CUR_D='0606'
mkdir $CUR_D && cd $CUR_D && mkdir Scale && cd Scale && mkdir Builder && mkdir Mono
cd .. && mkdir Trajectory && cd Trajectory && mkdir Builder && mkdir Mono
cd /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/debug

for status in $(seq 1 5); do  
    ./mono_tum ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/TUM3.yaml \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning/$CUR_D \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
        /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household \
        0.6 0.6;
done

# for status in $(seq 1 30); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM3.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning/$CUR_D \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household\
#         ../Examples/RGB-D/associations/fr3_long_office_household.txt \
#         0.6 0.6;
# done


cd /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning
# mask_thres, conf thres
CUR_D='0209'
mkdir $CUR_D && cd $CUR_D && mkdir Scale && cd Scale && mkdir Builder && mkdir Mono
cd .. && mkdir Trajectory && cd Trajectory && mkdir Builder && mkdir Mono
cd /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/debug

for status in $(seq 1 5); do  
    ./mono_tum ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/TUM3.yaml \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning/$CUR_D \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
        /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household \
        0.2 0.9;
done

# for status in $(seq 1 30); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM3.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning/$CUR_D \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household\
#         ../Examples/RGB-D/associations/fr3_long_office_household.txt \
#         0.2 0.9;
# done


cd /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning
# mask_thres, conf thres
CUR_D='0409'
mkdir $CUR_D && cd $CUR_D && mkdir Scale && cd Scale && mkdir Builder && mkdir Mono
cd .. && mkdir Trajectory && cd Trajectory && mkdir Builder && mkdir Mono
cd /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/debug

for status in $(seq 1 11); do  
    ./mono_tum ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/TUM3.yaml \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning/$CUR_D \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
        /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household \
        0.4 0.9;
done

# for status in $(seq 1 30); do  
#     ./rgbd_tum \
#         ../Vocabulary/ORBvoc.txt \
#         ../Examples/RGB-D/TUM3.yaml \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning/$CUR_D \
#         /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
#         /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household\
#         ../Examples/RGB-D/associations/fr3_long_office_household.txt \
#         0.4 0.9;
# done


cd /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning
# mask_thres, conf thres
CUR_D='0809'
mkdir $CUR_D && cd $CUR_D && mkdir Scale && cd Scale && mkdir Builder && mkdir Mono
cd .. && mkdir Trajectory && cd Trajectory && mkdir Builder && mkdir Mono
cd /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/debug

for status in $(seq 1 5); do  
    ./mono_tum ../Vocabulary/ORBvoc.txt \
        ../Examples/Monocular/TUM3.yaml \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning/$CUR_D \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
        /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household \
        0.8 0.9;
done

for status in $(seq 1 10); do  
    ./rgbd_tum \
        ../Vocabulary/ORBvoc.txt \
        ../Examples/RGB-D/TUM3.yaml \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2_tuning/ObjectSLAM/ObjLib/fr3_long_office_tuning/$CUR_D \
        /mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/model/ \
        /mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household\
        ../Examples/RGB-D/associations/fr3_long_office_household.txt \
        0.8 0.9;
done