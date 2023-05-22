# PULP-Detector

Authors:*Lorenzo Lamberti* <lorenzo.lamberti@unibo.it>
        *Luca Bompani* <luca.bompani5@unibo.it>
        *Manuele Rusci* <manuele.rusci@kuleuven.be>
        *Daniele Palossi* <dpalossi@iis.ee.ethz.ch>
Copyright (C) 2023 ***University of Bologna, KU Leuven, ETH Zürich,***. All rights reserved.

<img style="float: left;" src="images/cover.png" width="100%">

### **Videos** 
- **Exploration and Detection**: [Demo](https://youtu.be/BTin8g0nyko) 

### **Citing**

If you use **PULP-Detector** in an academic or industrial context, please cite the following publications:

Publications: 
* *Bio-inspired Autonomous Exploration Policies with CNN-based Object Detection on Nano-drones* ([arXiv preprint](https://arxiv.org/abs/2301.12175) - [DATE conference](???))


~~~~
@misc{pulp-detector,
      title={Bio-inspired Autonomous Exploration Policies with CNN-based Object Detection on Nano-drones}, 
      author={Lorenzo Lamberti and Luca Bompani and Victor Javier Kartsch and Manuele Rusci and Daniele Palossi and Luca Benini},
      year={2023},
      eprint={2301.12175},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
~~~~

## 1. Introduction
### What is PULP-Detector ?
**PULP-Detector** is a nano-drone system that strives for both maximizing the exploration of a room while performing visual object detection.
The Exploration policies as implemented as lightweight and bio-inpired state machines.
The object detection CNN is based on the MobilenetV2-SSD network.
The drone performs obstacle avoidance thanks to Time-of-flight sensors.
The drone is completely autonomous -- **no human operator, no ad-hoc external signals, and no remote laptop!**

- **Software component:**
Object detection CNN: is a shallow convolutional neural network (CNN) composed of Mobilenet-v2 backbone plus the SSD (single-shot detector) heads.
It runs at 1.6-4.3 FPS onboard.

- **Hardware components:**
The hardware soul of PULP-Detector is an ultra-low power visual navigation module embodied by a pluggable PCB (called *shield* or *deck*) for the [Crazyflie 2.0](https://www.bitcraze.io/crazyflie-2/)/[2.1](https://www.bitcraze.io/crazyflie-2-1/) nano-drone. The shield features a Parallel Ultra-Low-Power (PULP) GAP8 System-on-Chip (SoC) from GreenWaves Technologies (GWT), an ultra-low power HiMax HBM01 camera, and off-chip Flash/DRAM memory; This pluggable PCB has evolved over time, from the [*PULP-Shield*](https://ieeexplore.ieee.org/document/8715489) , the first custom-made prototype version developed at ETH Zürich, and its commercial off-the-shelf evolution, the [*AI-deck*](https://store.bitcraze.io/products/ai-deck).



Summary of characteristics:

 - **Hardware:** [*AI-deck*](https://store.bitcraze.io/products/ai-deck)

- **Deep learning framework:** Tensorflow 1.15 ([Tensorflow Object detection API](??))

- **Quantization**: fixed-point 8 bits, fully automated with [NNTool](https://greenwaves-technologies.com/sdk-manuals/nn_quick_start_guide/)

- **Deployment**: fully automated with [AutoTiler](https://greenwaves-technologies.com/sdk-manuals/nn_quick_start_guide/)

We release here, as open source, all our code, hardware designs, datasets, and trained networks.



## REFS

[PULP Platform Youtube](https://www.youtube.com/c/PULPPlatform) channel (subscribe it!)

[PULP Platform Website](https://pulp-platform.org/).


# Tincan and bottle Detection

in this repo there are the training, evaluation and deployment scripts used for the deployment of an SSD based object detector, working on Bottles and tin-cans.

## Setting up the environment

To use this repo you will need to setup the python environment needed which can be done using the tensorflow1_15.yml file 
> conda env create -f tensorflow1_15.yml



 





 
