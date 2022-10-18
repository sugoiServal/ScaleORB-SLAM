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

""" 
tempScaleCorrector:
    take weighted objects scale lib and mono objects(one sequence)
    output: scale correction factor

"""


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
        
# a = 0        
# for key in RGBD_class_log:
#     a += len(RGBD_class_log[key])

    
if __name__ == '__main__':
    
    Trajectory_GroundTruth_file = "/mnt/d/projects/slam_matlab/tum_rgbd_dataset/rgbd_dataset_freiburg3_long_office_household/groundtruth.txt"
    directory = "/mnt/d/projects/ORB_SLAM2_CV4/ORB_SLAM2/ObjectSLAM/ObjLib/freiburg3_long_office/"
    items = ["0209","0409","0607","0608","0809"]

    # ! process RGBD scale library

    
    RGBD_Scale_dirStr = directory + "Scale/Builder/" 
    
    # parseScalesLibFromDir()
        # RGBD_class_log: all sample points
        # RGBD_scale_lib: average scale to each classes
    (RGBD_class_log, RGBD_scale_lib) = parseScalesLibFromDir(RGBD_Scale_dirStr)
    
    # plot RGBD library 
    plt_dir = directory + "plot/"
    plotRGBDlib(RGBD_class_log, plt_dir+"lib/")
    plt_align_dir = plt_dir + "align/"
    
    
    
    # ! process Mono experiment result
    mono_ScaleDirStr = directory + "Scale/Mono/"
    mono_ScaleDir = os.fsencode(mono_ScaleDirStr)
    
    RGBD_TrjdirStr = directory +  "Trajectory/Builder/"
    RGBD_Trjdir = os.fsencode(RGBD_TrjdirStr)
    

    results_scales = {}



    for mono_ScaleFile in os.listdir(mono_ScaleDir):
        mono_Scalefilename = os.fsdecode(mono_ScaleFile)
        if mono_Scalefilename.endswith(".txt") : 
            print("    --" + mono_Scalefilename)
            # for each mono scale file
            mono_ScalefileDir = os.path.join(mono_ScaleDirStr, mono_Scalefilename)  
            (mono_class_log, mono_KFTrjDir) = parseScalesFromOneRun_Mono(mono_ScalefileDir)
            
            # test the model on all RGBD ground truth 
            for RGBD_TrjFile in os.listdir(RGBD_Trjdir):
                RGBD_Trjfilename = os.fsdecode(RGBD_TrjFile)
                if RGBD_Trjfilename.endswith(".txt") : 
                                                      
                    RGBD_TrjfileDir = os.path.join(RGBD_TrjdirStr, RGBD_Trjfilename) # for groundtruth calculation
                    
                    # calculate groundtruth
                    try:
                        (_,_,_,GroundTruthscale) = getScaleFactorGroundTruth(RGBD_TrjfileDir, mono_KFTrjDir)
                    except:
                        continue
                    
                    # calculate modeled result
                    
                    # compare modeled result with groundtruth     
                    try:
                        W_scale = tempScaleCorrector_W2(mono_class_log, RGBD_scale_lib)   # calculate weighted(#MP & #appear_in_lib) correction                                
                    except:
                        continue
                    #W_scale = tempScaleCorrector_W2(mono_class_log, RGBD_scale_lib)   # calculate weighted(#MP & #appear_in_lib) correction                 
                    WMP_scale = tempScaleCorrector_WMP(mono_class_log, RGBD_scale_lib)   # calculate weighted(#MP) correction 
                    NW_scale = tempScaleCorrector_NW(mono_class_log, RGBD_scale_lib)    # calculate unweighted correction
                                      
                    if not(mono_ScaleFile in results_scales):
                        results_scales[mono_ScaleFile] = [(GroundTruthscale, abs(W_scale-GroundTruthscale), abs(WMP_scale-GroundTruthscale), abs(NW_scale-GroundTruthscale))]
                    else:
                        results_scales[mono_ScaleFile].append((GroundTruthscale, abs(W_scale-GroundTruthscale), abs(WMP_scale-GroundTruthscale), abs(NW_scale-GroundTruthscale)))
                 
    # plots                
    plt_res = [[] for i in range(4)]      
    for (mono_ScaleFile, data) in results_scales.items():
        print (mono_ScaleFile)
    
        diffs = list(zip(*data)) 
        print( "  **--GroundTruth (avg): " + str(np.average(diffs[0])))
        print( "    --W_scale abs diff (avg): " + str(np.average(diffs[1])))
        print( "    --WMP_scale abs diff (avg): " + str(np.average(diffs[2])))
        print( "    --NW_scale abs diff (avg): " + str(np.average(diffs[3])))       
        plt_res[0].append(np.average(diffs[0]))
        plt_res[1].append(np.average(diffs[1]))
        plt_res[2].append(np.average(diffs[2]))
        plt_res[3].append(np.average(diffs[3]))
    
    # plot whole result
    fig, ax = plt.subplots()
    #plt.figure()
    
    ax.plot(np.arange(start=1, stop = len(plt_res[0])+1), plt_res[0],  'go--', label='Groundtruth')
    ax.plot(np.arange(start=1, stop = len(plt_res[0])+1), plt_res[1],  'ro--', label='(abs dist) weighted #MP and #appear')
    ax.plot(np.arange(start=1, stop = len(plt_res[0])+1), plt_res[2],  'bo--', label='(abs dist) weighted #MP')
    ax.plot(np.arange(start=1, stop = len(plt_res[0])+1), plt_res[3],  'ko--', label='(abs dist) no weighted')
    
    plt.xticks(np.arange(1, len(plt_res[0])+1, 1))
    means = [np.average(plt_res[i]) for i in range(4)]
    ax.hlines(means, xmin = 0, xmax = len(plt_res[0]), colors=['g', 'r', 'b', 'k'])
    ax.text(0, 2.6, 'means', ha ='left', va ='center')

    ax.set_xlabel("Monocular runs")
    ax.set_ylabel("Groundtruth scale correction/ abs distance to Groundtruth")

    ax.set_title(" Compare Modeled Scales and RGBD Scales, sample = " + str(len(results_scales[random.choice(list(results_scales.keys()))])))
    ax.legend(shadow=False)
    plt.show()
    fig.savefig(plt_dir + "experiment_result.jpg", dpi=500)



    # RMSE table 
    
    # selects for scale RMSEs
    offset = 0
    max_difference = 0.02 
    save = True  # save aligned second trajectory to disk (format: stamp2 x2 y2 z2)
    save_associations =  True  # save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)
    verbose = False # print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)

    

    
    result_RMSEs = {"mono_est":[], "mono_opt":[], "RGBD_trj":[], "mono_origin":[]}
    plot_align_rgbd = plt_align_dir+ "long_office_align"+"(RGBD).jpg"
    plot_align_mono = plt_align_dir+ "long_office_align"+"(Mono).jpg"   
    plot_align_mono_opt = plt_align_dir+ "long_office_align"+"(Mono_opt).jpg"   
    plot_align_mono_origin = plt_align_dir+ "long_office_align"+"(mono_org).jpg"   
    

    results_scales = []
    
    for mono_ScaleFile in os.listdir(mono_ScaleDir):
        mono_Scalefilename = os.fsdecode(mono_ScaleFile)
        if mono_Scalefilename.endswith(".txt") : 
            print("    --" + mono_Scalefilename)
            # for each mono scale file
            mono_ScalefileDir = os.path.join(mono_ScaleDirStr, mono_Scalefilename)  
            (mono_class_log, mono_KFTrjDir) = parseScalesFromOneRun_Mono(mono_ScalefileDir)
            
            
            # optimized monocular scale
            (_,_,_,mono_opt_scale) = getScaleFactorGroundTruth(Trajectory_GroundTruth_file, mono_KFTrjDir)
                       
            # estimated monocular scale
            try:
                W_scale = tempScaleCorrector_W2(mono_class_log, RGBD_scale_lib)   # calculate weighted(#MP & #appear_in_lib) correction                                
            except:
                continue
            
            results_scales.append(W_scale/mono_opt_scale)
            
            # plot align once
            m_result_RMSEs = calculateAlign(Trajectory_GroundTruth_file, mono_KFTrjDir, W_scale, plot = plot_align_mono)
            m_result_RMSEs_opt = calculateAlign(Trajectory_GroundTruth_file, mono_KFTrjDir, mono_opt_scale, plot = plot_align_mono_opt)
            m_result_RMSEs_origin = calculateAlign(Trajectory_GroundTruth_file, mono_KFTrjDir, 1, plot = plot_align_mono_origin)
            
            result_RMSEs["mono_est"].append(m_result_RMSEs)
            result_RMSEs["mono_opt"].append(m_result_RMSEs_opt)
            result_RMSEs["mono_origin"].append(m_result_RMSEs_origin)
            
            plot_align_mono = None
            plot_align_mono_opt  = None
            plot_align_mono_origin = None


    for RGBD_TrjFile in os.listdir(RGBD_Trjdir):
        RGBD_Trjfilename = os.fsdecode(RGBD_TrjFile)
        if RGBD_Trjfilename.endswith(".txt") : 
            print("    --" + RGBD_Trjfilename)                                  
            RGBD_TrjfileDir = os.path.join(RGBD_TrjdirStr, RGBD_Trjfilename) # for groundtruth calculation        
            r_result_RMSEs = calculateAlign(Trajectory_GroundTruth_file, RGBD_TrjfileDir, plot = plot_align_rgbd)
            result_RMSEs["RGBD_trj"].append(r_result_RMSEs)
            plot_align_rgbd = None
            
    mono_est = np.array(result_RMSEs["mono_est"])
    mono_opt = np.array(result_RMSEs["mono_opt"])
    RGBD_trj = np.array(result_RMSEs["RGBD_trj"])
    mono_origin = np.array(result_RMSEs["mono_origin"])
    
    results_scales = np.array(results_scales)        
    geo_mean_scale = results_scales.prod()**(1.0/len(results_scales))
    zz = {"mono_est": mono_est, "mono_opt":mono_opt, "RGBD_trj": RGBD_trj, "mono_origin":mono_origin, "scaleQuo":geo_mean_scale}
    bb = ["mono_est", "mono_opt", "RGBD_trj", "mono_origin"]
    for item in bb:
        avg = np.mean(zz[item], axis = 0)[0]
        zz[item] = avg
    zz["improv"] = (zz["mono_origin"] - zz["mono_est"])/zz["mono_origin"] 
