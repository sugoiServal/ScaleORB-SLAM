import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression


FILE_RGBD = '/mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/Trajectory/Builder/(KFTrj)rgbd_dataset_freiburg3_long_office_household_2021_11_01_15_37.txt'
FILE_Monocular = '/mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/Trajectory/Mono/(KFTrj)rgbd_dataset_freiburg3_long_office_household_2021_11_01_15_41.txt'


if __name__ == '__main__':
    trajectoryRGBD = np.loadtxt(FILE_RGBD)
    trajectoryMonocular = np.loadtxt(FILE_Monocular)





    # plots 
    # for i in range(1, trajectoryMonocular.shape[1]):
    #     fig = plt.figure()
    #     ax1 = fig.add_subplot(111)
    #     ax1.scatter(trajectoryRGBD[:,0], trajectoryRGBD[:,i], c='b', marker="s")
    #     ax1.scatter(trajectoryMonocular[:,0], trajectoryMonocular[:,i]*2.75, c='r', marker="o")
    #     ax1.set_title('xyz'+str(i))
    #     plt.show()
    #     print(i)
    #     input("Press Enter to continue...")

        