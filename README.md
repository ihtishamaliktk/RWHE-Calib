# Methods for Simultaneous Robot-World Hand-Eye Calibration: A Comparative Study

# Introduction
We propose a collection of robot-world/hand-eye calibration methods to establish a geometrical relationship between robot, camera, and its environment. We examine the calibration problem from two alternative geometrical interpretations, namely hand-eye and robot-world-hand-eye as shown in the figure. The study analyses the effects of approaching the objective as pose error and reprojection error minimization problem.

![mainFig](https://user-images.githubusercontent.com/32157027/58380789-21da9e00-7fbe-11e9-9cf7-e81a5b515652.png)

**_Figure_**: Formulations relating geometrical transformation for calibration (a) Hand-Eye Calibration (b) Robot-World-Hand-Eye Calibration

The datasets are provided as a part of [Centre for Immersive Visual Technologies (CIVIT)](http://www.tut.fi/civit/) initiative to provide open-access data.


# How to cite
In review, until acceptance, you can cite this page.

# Dependencies
The code is dependent on Matlab and some of its toolkits such as Computer Vision Toolbox.

# How to use
Simply run the _main.m_ file to reproduce the results provided in Table.3 of the publication. The code by default runs the kuka_2 dataset. To use a different dataset, specify the name of the dataset you want to use in the _main.m_ file e.g. _DatasetName=’CS_synthetic_3’_. The relevant info (square size, Ground Truth if available, poses etc) will be loaded automatically. In order to use your own dataset, follow the pattern of data provided and the name of your dataset in the main file. Upon the request of square size, give it in meters.

# Contact
Ihtisham Ali

Tampere University, Finland

ihtisham.ali@tuni.fi 

https://www.researchgate.net/profile/Ihtisham_Ali3


