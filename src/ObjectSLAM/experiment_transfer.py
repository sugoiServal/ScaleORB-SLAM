#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 31 18:20:45 2021

@author: aabad
"""
import numpy as np
import matplotlib.pyplot as plt 
import os
import random 
from groundtruthScale import *

bug = ["teddy bear",
"hair drier",
"cell phone",
"dining table",
"potted plant",
"hot dog",
"wine glass",
"tennis racket",
"baseball bat",
"baseball glove",
"sports ball",
"parking meter",
"traffic light",
"fire hydrant",
"stop sign"]




def parseScalesLibFromDir(RGBD_Scales_dir):
    """ 
    Input: 
        fileDir(str): RGBD scale files directiory 
    OutPut 
        RGBD_class_log(dict): {objectClass, list_of_scales}
        the library of RGBD object scale
    """
    
    directory = os.fsencode(RGBD_Scales_dir)
    RGBD_class_log = {}
    
    for file in os.listdir(directory):
         filename = os.fsdecode(file)
         if filename.endswith(".txt") : 
            fileDir = os.path.join(RGBD_Scales_dir, filename)   
            list_of_lists = []
            with open(fileDir) as f:
               for line in f:
                   inner_list = [elt.strip() for elt in line.split(' ')]
                   list_of_lists.append(inner_list)

            Objects = list_of_lists[4:]
            KFTrjdir = list_of_lists[2]
            camTrjdir = list_of_lists[3]
                
            for obj in Objects:
                if ((obj[0] + " " + obj[1]) in bug):
                    className = obj[0] + " " + obj[1]
                    nMP_scale = (int(obj[3]), float(obj[-1]))
                else :
                    className = obj[0] 
                    nMP_scale = (int(obj[2]), float(obj[-1]))
        
                if not(className in RGBD_class_log):
                    RGBD_class_log[className] = [nMP_scale]
                else:
                    RGBD_class_log[className].append(nMP_scale)
                    
            
            RGBD_scale_lib = {}
            for (objClass,data) in RGBD_class_log.items():
                # calculate averages
                nMPs_scales = list(zip(*data))
                wmean = np.average(nMPs_scales[1], weights = nMPs_scales[0])
                RGBD_scale_lib[objClass] = (len(data), wmean)
      
    return RGBD_class_log, RGBD_scale_lib

def parseScalesLibFromkDirs(RGBD_Scales_dirs):
    """ 
    Input: 
        fileDir(str): set of RGBD scale files directiories
    OutPut 
        RGBD_class_log(dict): {objectClass, list_of_scales}
        the library of RGBD object scale
    """

    
    RGBD_class_log = {}
    for RGBD_Scales_dir in RGBD_Scales_dirs:
        directory = os.fsencode(RGBD_Scales_dir)      
        for file in os.listdir(directory):
             filename = os.fsdecode(file)
             if filename.endswith(".txt") : 
                fileDir = os.path.join(RGBD_Scales_dir, filename)   
                list_of_lists = []
                with open(fileDir) as f:
                   for line in f:
                       inner_list = [elt.strip() for elt in line.split(' ')]
                       list_of_lists.append(inner_list)
    
                Objects = list_of_lists[4:]
                KFTrjdir = list_of_lists[2]
                camTrjdir = list_of_lists[3]
                    
                for obj in Objects:
                    if ((obj[0] + " " + obj[1]) in bug):
                        className = obj[0] + " " + obj[1]
                        nMP_scale = (int(obj[3]), float(obj[-1]))
                    else :
                        className = obj[0] 
                        nMP_scale = (int(obj[2]), float(obj[-1]))
            
                    if not(className in RGBD_class_log):
                        RGBD_class_log[className] = [nMP_scale]
                    else:
                        RGBD_class_log[className].append(nMP_scale)
                        
                
    RGBD_scale_lib = {}
    for (objClass,data) in RGBD_class_log.items():
        # calculate averages
        nMPs_scales = list(zip(*data))
        wmean = np.average(nMPs_scales[1], weights = nMPs_scales[0])
        RGBD_scale_lib[objClass] = (len(data), wmean)
          
    return RGBD_class_log, RGBD_scale_lib




def plotRGBDlib(RGBD_class_log, plot_dir = None):

    for (objClass,data) in RGBD_class_log.items():
        # calculate averages
        nMPs_scales = list(zip(*data))
        mean = np.average(nMPs_scales[1])
        wmean = np.average(nMPs_scales[1], weights = nMPs_scales[0])
        # plot
        fig, ax = plt.subplots()
        ax.scatter(nMPs_scales[0], nMPs_scales[1], c=nMPs_scales[0], cmap='gray_r', marker = ",", linewidth = 0.1)
        ax.hlines([mean, wmean], xmin = min(nMPs_scales[0]), xmax = max(nMPs_scales[0]), colors=['r', 'g'], label = ["mean", "weighted mean"])
        ax.text(min(nMPs_scales[0]), mean, 'mean', ha ='left', va ='center')
        ax.text(min(nMPs_scales[0]), wmean, 'weighted mean', ha ='left', va ='center')
        ax.set_xlabel("#MP")
        ax.set_ylabel("Object Scale")
        ax.set_title(objClass + " (RGBD lib), sample size = " + str(len(nMPs_scales[0])))
        plt.show()
        if plot_dir is not None:
            fig.savefig(plot_dir + objClass + "_lib.jpg")
        
        
def parseScalesFromOneRun_Mono(fileDir):
    """ 
    # process single file
    Input: 
        fileDir(str): path to a Monocular scale log
    OutPut 
        Mono_class_log(dict): {objectClass, list_of_scales}: list_of_scales: tuple(num_MPs, scale)
        mono_KFTrjDir(str): path to the Monocular run trajectory, for groundtruth calculation 
    """

    mono_class_log = {}
    list_of_lists = []
    with open(fileDir) as f:
       for line in f:
           inner_list = [elt.strip() for elt in line.split(' ')]
           list_of_lists.append(inner_list)

    Objects = list_of_lists[3:]
    KFTrjdir = list_of_lists[2][0]
        
    for obj in Objects:
        if ((obj[0] + " " + obj[1]) in bug):
            className = obj[0] + " " + obj[1]
            nMP_scale = (float(obj[3]), float(obj[-1]))
        else :
            className = obj[0] 
            nMP_scale = (float(obj[2]), float(obj[-1]))

        if not(className in mono_class_log):
            mono_class_log[className] = [nMP_scale]
        else:
            mono_class_log[className].append(nMP_scale)
            
    
    return mono_class_log, KFTrjdir
    
def tempScaleCorrector_NW(mono_class_log, RGBD_scale_lib):
    scales = []
    for (className, data) in mono_class_log.items():
        for instance in data: # iterate in list of instance
            scales.append(RGBD_scale_lib[className][1]/instance[1])
    scale = np.average(scales)           
    return scale
def tempScaleCorrector_W2(mono_class_log, RGBD_scale_lib):
    scale_cate = {}
    N_all_samples = 0.0
    
    # weight object categories according to #Object_appear_in_library
    for (className, data) in mono_class_log.items():
        N_all_samples += RGBD_scale_lib[className][0]
        
    for (className, data) in mono_class_log.items():
        for instances in data:                    
            if not(className in scale_cate):
                scale_cate[className] = [(instances[0]*RGBD_scale_lib[className][0]/N_all_samples, instances[1])]      
            else:     
                scale_cate[className].append((instances[0]*RGBD_scale_lib[className][0]/N_all_samples, instances[1]))
    
    # weight scale factor combining #MP and #Object_appear_in_library
    all_obj = []
    for (className, data) in scale_cate.items():
        for instance in data:
            all_obj.append((instance[0], RGBD_scale_lib[className][1]/instance[1]))
    
    weight_scales = list(zip(*all_obj))      
    wmean = np.average(weight_scales[1], weights = weight_scales[0])
            
            
    return wmean

def tempScaleCorrector_WMP(mono_class_log, RGBD_scale_lib):

    # weight scale factor combining #MP and #Object_appear_in_library
    all_obj = []
    for (className, data) in mono_class_log.items():
        for instance in data:
            all_obj.append((instance[0], RGBD_scale_lib[className][1]/instance[1]))   
    weight_scales = list(zip(*all_obj))      
    wmean = np.average(weight_scales[1], weights = weight_scales[0])
            
            
    #for (className, data) in scale_cate.items():
    #   for instance in data: 
    #        scales.append((RGBD_scale_lib[className][1]/instance[1])*(instance[0]/N_all_weight))
    #scale = np.average(scales)         
            
    return wmean        
        
        


    
if __name__ == '__main__':
    
    
    target_sequences = ["freiburg1_desk", 
                        "freiburg1_room", 
                        "freiburg1_xyz", 
                        "freiburg3_long_office"]
    groundtruths = ["rgbd_dataset_freiburg1_desk", 
                    "rgbd_dataset_freiburg1_room", 
                    "rgbd_dataset_freiburg1_xyz",             
                    "rgbd_dataset_freiburg3_long_office_household"]
    

    root = "/mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/"
    # build object library from sequences excluding the testing sequence
    for (test_sequence, groundtruth) in zip(target_sequences, groundtruths):
        print(test_sequence)
        if test_sequence != 'rgbd_dataset_freiburg3_long_office_household':
            continue
        # setup test file
        Trajectory_GroundTruth_file = "/mnt/d/projects/slam_matlab/tum_rgbd_dataset/" \
                                + groundtruth + "/groundtruth.txt"
        test_directory = root + test_sequence + "/"
        plt_dir = test_directory + "plot/transfer_inc/"
        plt_align_dir = plt_dir + "align/"

        # setup lib
        #lib_dirs = {root+seq+'/Scale/Builder/' for seq in target_sequences if seq is not test_sequence}
        lib_dirs = {root+seq+'/Scale/Builder/' for seq in target_sequences}
        (RGBD_class_log, RGBD_scale_lib) = parseScalesLibFromkDirs(lib_dirs)
        plotRGBDlib(RGBD_class_log, plt_dir+"lib/")
        
        


    
    
        # ! process Mono experiment result
        mono_ScaleDirStr = test_directory + "Scale/Mono/"
        mono_ScaleDir = os.fsencode(mono_ScaleDirStr)
        
        RGBD_TrjdirStr = test_directory +  "Trajectory/Builder/"
        RGBD_Trjdir = os.fsencode(RGBD_TrjdirStr)
        

        results_scales = []

    
        min_RMSE = 100
        min_info = []
        result_RMSEs = {"mono_est":[], "mono_opt":[], "RGBD_trj":[], "mono_origin":[]}
        
        # selects for scale RMSEs
        offset = 0
        max_difference = 0.02 
        save = True  # save aligned second trajectory to disk (format: stamp2 x2 y2 z2)
        save_associations =  True  # save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)
        verbose = False # print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)
      
        # align
        plot_align_rgbd = plt_align_dir+ "long_office_align"+"(RGBD).jpg"
        plot_align_mono = plt_align_dir+ "long_office_align"+"(Mono).jpg"   
        plot_align_mono_opt = plt_align_dir+ "long_office_align"+"(Mono_opt).jpg"   
        plot_align_mono_origin = plt_align_dir+ "long_office_align"+"(mono_org).jpg"   
        
        # 30 mono scales
        for mono_ScaleFile in os.listdir(mono_ScaleDir):
            mono_Scalefilename = os.fsdecode(mono_ScaleFile)
            if mono_Scalefilename.endswith(".txt") : 
                print("    --" + mono_Scalefilename)
                
                # for each mono scale file
                mono_ScalefileDir = os.path.join(mono_ScaleDirStr, mono_Scalefilename)  
                (mono_class_log, mono_KFTrjDir) = parseScalesFromOneRun_Mono(mono_ScalefileDir)
            
                # calculate groundtruth
                try:
                    (_,_,_,GroundTruthscale) = getScaleFactorGroundTruth(Trajectory_GroundTruth_file, mono_KFTrjDir)
                except:
                    continue
                
                # calculate modeled scales   
                # compare modeled result with groundtruth    
                try:
                    W_scale = tempScaleCorrector_W2(mono_class_log, RGBD_scale_lib)   # calculate weighted(#MP & #appear_in_lib) correction                                
                except:
                    continue
                #WMP_scale = tempScaleCorrector_WMP(mono_class_log, RGBD_scale_lib)   # calculate weighted(#MP) correction 
                #NW_scale = tempScaleCorrector_NW(mono_class_log, RGBD_scale_lib)    # calculate unweighted correction                               
                results_scales.append(W_scale/GroundTruthscale)
            
                # compare RMSEs
                m_result_RMSEs = calculateAlign(Trajectory_GroundTruth_file, mono_KFTrjDir, W_scale, plot = None)        
                m_result_RMSEs_opt = calculateAlign(Trajectory_GroundTruth_file, mono_KFTrjDir, GroundTruthscale, plot = None)
                m_result_RMSEs_origin = calculateAlign(Trajectory_GroundTruth_file, mono_KFTrjDir, 1, plot = None)
                
                result_RMSEs["mono_est"].append(m_result_RMSEs)
                result_RMSEs["mono_opt"].append(m_result_RMSEs_opt)
                result_RMSEs["mono_origin"].append(m_result_RMSEs_origin)
                

        result_RMSEs["mono_est"].pop(21)        
        mono_est = np.mean(np.array(result_RMSEs["mono_est"]), axis = 0)
        mono_opt = np.mean(np.array(result_RMSEs["mono_opt"]), axis = 0)
        mono_origin = np.mean(np.array(result_RMSEs["mono_origin"]), axis = 0)
        # RGBD_trj = np.mean(np.array(result_RMSEs["RGBD_trj"]), axis = 0)
        
        
        results_scales = np.array(results_scales)        
        geo_mean_scale = results_scales.prod()**(1.0/len(results_scales))
        
        addon = [(mono_origin[0]-mono_est[0])/mono_origin[0], geo_mean_scale, 0, 0, 0, 0]
        export = np.stack((mono_est, mono_opt, mono_origin, addon))
        np.savetxt(plt_dir+'RMSEs.csv', export, delimiter=',')
