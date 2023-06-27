

# Tincan and bottle Detection

in this repo there are the training, evaluation and deployment scripts used for the deployment of an SSD based object detector, working on Bottles and tin-cans.

## Setup

To use this repo you will need to setup the python environment needed which can be done using the tensorflow1_15.yml file
> conda env create -f tensorflow1_15.yml

~~ ~~
# How to train a model in Tensorflow Obj Detection API

Take a look at [Official TensorFlow Object Detection API Tutorial](https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/index.html) to set up the environment and to have more insights.

Look also to the git repo [README](https://github.com/tensorflow/models/tree/master/research/object_detection) file to have more examples and tutorials.


### OpenImagesV4 Dataset
Open Images is a dataset of ~9M images annotated with image-level labels, object bounding boxes, object segmentation masks, visual relationships, and localized narratives. It contains a total of 16M bounding boxes for 600 object classes on 1.9M images, making it the largest existing dataset with object location annotations.

### Download a subset of OpenImagesV4 Dataset

There is a [GitHub Repo](https://github.com/EscVM/OIDv4_ToolKit) that allows to download all the images of this this dataset containing a specific class (and only that class!).The annotations of the images will include just labels and boxes for that class too (for example I downloaded just images of license plates).

Example of a command for executing OIDv4_ToolKit

`python3 main.py downloader --classes classes.txt --type_csv all`

> **--classes** : you specify what classes you want to download (write the corresponding label). if the class name has a space in it , like "polar bear", you must write it with **underscores** "polar_bear". To download multiple classes, you can create a classes.txt file (and give this to the --classes opt) in which each line corresponds to a class name.
>
>**--type_cvs** : you can select "train", "test", "validation", "all". Selecting "all" you will download 3 folders with images divided into train, valid and test sets (so you are downloading all the images available for your class)

we provide a simple data augmentation utility which attach a black stip of varing length to the rightmost side of an image




### TFRecord generation

There is a [GitHub Repo](https://github.com/zamblauskas/oidv4-toolkit-tfrecord-generator/blob/master/README.md) that gives an easy script for generating the TFRecords of the OIDv4 subset downloaded.

`python generate-tfrecord.py --classes_file=classes.txt --class_descriptions_file=class-descriptions-boxable.csv --annotations_file=train-annotations-bbox.csv --images_dir=train_images_folder/ --output_file=train.tfrecord`

**IMPORTANT:** Here the classes.txt file doesn't want underscores instead of white spaces!!! (unlike the dataset downloader OIDv4ToolKit!)
For example, you have to write again "polar bear" instead of "polar_bear".

### Label Map

TensorFlow requires a label map, which namely maps each of the used labels to an integer values. This label map is used both by the training and detection processes.

The Label Maps for standard datasets can be found in the tensorflowAPI repository at `models/research/object_detection/data`

The classes included in the label map should be exactly the ones that you are training on. If you set to train on just 1 class, then leave only that class in the label_map.pbtxt file with `id: 1`.

### Configuration

How to setup the training/config_file.config file.

You can find all the default config files in `models/research/object_detection/samples/configs` folder. Make sure to set correctly all the paths (search for "PATH_TO_BE_CONFIGURED" to find the fields).

More details on the essential fields to set can be found [here](https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/training.html#configuring-a-training-pipeline) and at the "Useful stuff" paragraph.

we provide the configurations files for both quantization aware training and normal training.


### Metrics

All the metrics available are declared in `models/research/object_detection/eval_util.py`.

By default, [COCO metrics](http://cocodataset.org/#detection-eval) are used.

You can look at the [Tensorflow model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf1_detection_zoo.md) for reference accuracy of the mAP metric over COCO and OpenImages dataset

### Training

Use the `train_eval_model_main.py` script to train your model. It will save checkpoints and tensorflow events that will keep trace of the training process.


Example training command:

`python train_eval_model_main.py --pipeline_config_path=config/ssd_mobilenet_v2_oid_v4.config --model_dir=training/ --alsologtostderr`

> **--model_dir** : where checkpoints ad tensorboard logs will be saved


### Testing
Use the `train_eval_model_main.py` script to test your model.

Example testion command:
```python train_eval_model_main.py --pipeline_config_path=configs/ssd_mobilenet_v2_oid_v4_copy.config --checkpoint_dir=training/  --run_once```

> **--checkpoint_dir** : directory where the checkpoints have been saved by default it uses the last.


### FROZEN GRAPH EXPORT

The checkpoints produced by the training of the neural network requires to be exported in a format that can  be converted in  tflite for later deployment. First of all you'll need to use the export_tflite_ssd_graph.py python script which you can find in the training directory . An example
`python export_tflite_ssd_graph.py --trained_checkpoint_prefix ./model.ckpt-### --output_directory ./ --pipeline_config_path ./configs/ssd_mobilenet_v2_oid_v4.config`


it exports a file called tflite_graph.pb in the directory indicated (the current one)

## TFLITE CONVERSION

the conversion of the frozen graph can be done using the tflite_convert command. usage example:

`tflite_convert --graph_def_file=tflite_graph.pb --output_file=graph.tflite --inference_type=QUANTIZED_UINT8 --input_arrays=normalized_input_image_tensor --output_arrays=TFLite_Detection_PostProcess,TFLite_Detection_PostProcess:1,TFLite_Detection_PostProcess:2,TFLite_Detection_PostProcess:3 --mean_values=128 --std_dev_values=127.5 --input_shapes=1,240,320,3 --allow_custom_ops --inference_input_type=QUANTIZED_UINT8`
