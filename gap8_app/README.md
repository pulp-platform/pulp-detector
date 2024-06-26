# Single shot detector (SSD) CNN

In this repo, there are the training, evaluation, and deployment scripts used for the deployment of an SSD-based object detector.

Objects our CNN can detect: Tin cans and bottles

## Setup

There are two steps for this set up first is to set up the python environment needed which can be done using the tensorflow1_15.yml file 
> conda env create -f tensorflow1_15.yml

then you need to install the gap_sdk, which can be installed following the instruction of the official [repository](https://github.com/GreenWaves-Technologies/gap_sdk/tree/release-v4.7.0). 

Gap-sdk version used in this work: 4.7

 

## Deployment on the nano-drone

This section shows the commands necessary for the deployment of a quantized neural network on the GAP8 SoC, which is on the AI-Deck of the nano drone.

### How to run the CNN on GAP8
 
You need to source the correct configuration this can be done with the following command: 

`source path/to/gap_sdk/configs/ai_deck.sh`

Once this is done, you need to export the GAPY_OPENOCD_CABLE, which can be done with the command

`export GAPY_OPENOCD_CABLE=path/to/gap_sdk/utils/gap8-openocd/tcl/interface/ftdi/olimex-arm-usb-ocd-h.cfg`

the last step for the deployment is to use the :
`make clean all run`
and run the application on the drone.

### Visualize camera stream and detections

To  visualize the detections being performed by the drone you need first to connect to the Bitcraze AI-deck example wi-fi network.

The script requires the gi package, which can be installed following this [GUIDE](https://pygobject.readthedocs.io/en/latest/getting_started.html#ubuntu-getting-started)

 Then you need to run the viewer_custom.py with :
```python viewer_custom.py ```





 
