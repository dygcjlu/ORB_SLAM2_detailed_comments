#!/bin/bash

echo "start...."

export LD_LIBRARY_PATH=/media/jason/Data2/3d/VTK/build/install/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/media/jason/Data2/3rd_party/pcl/build_vtk7/install/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/media/jason/Data2/3rd_party/Pangolin/build/install/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/media/jason/Data2/opencv/opencv_3_4_5/build_release/install/lib:$LD_LIBRARY_PATH


/media/jason/Data2/3d/stereo/mygithub/ORB_SLAM2_detailed_comments/Examples/Stereo/stereo_euroc \
/media/jason/Data2/3d/stereo/mygithub/ORB_SLAM2_detailed_comments/Vocabulary/ORBvoc.txt \
/media/jason/Data2/3d/stereo/mygithub/ORB_SLAM2_detailed_comments/Examples/Stereo/custom.yaml \
/media/jason/Data2/datasets/3d/mine/0410/5/for_slam/mav0/cam0/data \
/media/jason/Data2/datasets/3d/mine/0410/5/for_slam/mav0/cam1/data \
/media/jason/Data2/datasets/3d/mine/0410/5/for_slam/custom_data.txt \
/media/jason/Data2/datasets/3d/mine/0410/5/for_slam/ 1\


echo "SFM Done...."



