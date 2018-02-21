##

RPG Monocular Pose Estimator
======================

Disclaimer and License
---------------

The RPG Monocular Pose Estimator is recommended to be used with ROS-Kinetic and Ubuntu 16.04.
This is research code, expect that it changes often (or not at all) and any fitness for a particular purpose is disclaimed.

The source code is released under a GPL licence. Please contact the authors for a commercial license.


Package Summary
---------------

The RPG Monocular Pose Estimator uses infrared LEDs mounted on a target object and a camera with an infrared-pass filter to estimate the pose of an object.

The positions of the LEDs on the target object are provided by the user in a YAML configuration file. The LEDs are detected in the image, and a pose estimate for the target object is subsequently calculated.

### Publication

If you use this work in an academic context, please cite the following [ICRA 2014 publication](http://rpg.ifi.uzh.ch/docs/ICRA14_Faessler.pdf):

M. Faessler, E. Mueggler, K. Schwabe, D. Scaramuzza: 
**A Monocular Pose Estimation System based on Infrared LEDs.**
IEEE International Conference on Robotics and Automation (ICRA), Hong Kong, 2014.

    @InProceedings{Faessler14icra,
      author        = {Matthias Faessler and Elias Mueggler and Karl Schwabe and
                      Davide Scaramuzza},
      title         = {A Monocular Pose Estimation System based on Infrared {LED}s},
      booktitle     = {{IEEE} Int. Conf. Robot. Autom. (ICRA)},
      year          = 2014,
      pages         = {907--913},
      doi           = {10.1109/ICRA.2014.6906962}
    }

Watch the [video](http://www.youtube.com/watch?v=8Ui3MoOxcPQ) demonstrating the RPG Monocular Pose Estimator:   
[![ RPG Monocular Pose Estimator Video](http://img.youtube.com/vi/8Ui3MoOxcPQ/0.jpg)](http://www.youtube.com/watch?v=8Ui3MoOxcPQ)


Installation
------------

### Installation of the package

#### System Dependencies

The RPG Monocular Pose Estimator is built on the Robotic Operating System (ROS). In order to install the package, ROS has to be installed.

- In order to install the Robot Operating System (ROS), please follow the instructions provided in the [link](http://wiki.ros.org).
- Make sure you have properly set up a ROS catkin workspace as described [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Additionally, the RPG Monocular Pose Estimator makes use of [OpenCV](http://opencv.org) for image processing and the [Eigen](http://eigen.tuxfamily.org) linear algebra library. **These should come preinstalled with ROS**, however, if the dependency is missing they can be installed from their respective websites:

- To install OpenCV, follow the installation instructions provided on the OpenCV [website](http://opencv.org).

- To install the Eigen linear algebra library, follow the installation instructions provided on the Eigen [website](http://eigen.tuxfamily.org).

#### Main Installation

In order to install the RPG Monocular Pose Estimator and its dependencies, first install [vcstool](https://github.com/dirk-thomas/vcstool):

    sudo apt-get install python-vcstool

Then clone the latest version from our *GitHub* repository into your catkin workspace, get the dependencies with the vcstool and compile the packages using ROS.:

    cd catkin_workspace/src
    git clone https://github.com/uzh-rpg/rpg_monocular_pose_estimator.git
    vcs-import < rpg_monocular_pose_estimator/dependencies.yaml
    catkin_make

Source your catkin workspace after this.

### Building the Documentation

The RPG Monocular Pose Estimator makes use of [Doxygen](http://www.doxygen.org/‎) to produce its documentation. Please ensure that you have the latest version of Doxygen which can be installed with:

    sudo apt-get install doxygen

The Doxygen configuration file is located in the *doxygen_documentation* folder within the RPG Monocular Pose Estimator directory. 

Change to the doxygen_documentation directory:

    roscd monocular_pose_estimator/../doxygen_documentation

To produce the documentation, run Doxygen on the configuration file:

    doxygen doxygen_config_file.cfg

Open the index.html file in *html* directory that has been produced.

Test Installation on Basic Dataset
----------------------------------

In order to test the installation on a data set, download the data set from [here](http://rpg.ifi.uzh.ch/data/monocular-pose-estimator-data.tar.gz), and follow these instructions.

1.    Download and Untar a sample ROS bag file

          roscd monocular_pose_estimator
          mkdir bags
          cd bags
          wget http://rpg.ifi.uzh.ch/data/monocular-pose-estimator-data.tar.gz
          tar -zxvf monocular-pose-estimator-data.tar.gz
          rm monocular-pose-estimator-data.tar.gz

2.    Launch the demo launch file using

          roslaunch monocular_pose_estimator demo.launch

3.    You should see a visualisation of the system: detected LEDs should be circled in red, the region of interest in the image that is being processed should be bounded by a blue rectangle, and the orientation of the tracked object will be represented by the red-green-blue trivector located at the origin of the traget object's coordinate frame.


Basic Usage
-----------

The program makes use of a number of parameters to estimate the pose of the tracked object. These include the location of the LEDs on the object, the intrinsic parameters of the camera, and various runtime prameters, such as thresholding values.

#### Setting up the Marker Positions Parameter File

In order to predict the pose of the target object, the RPG Monocular Pose Estimator needs to know the positions of the LEDs on the object in the object's frame of reference. These positions are given to the program using a YAML file located within the `roscd monocular_pose_estimator/marker_positions` folder.

By default, the file is called `marker_positions.YAML` and has the following format.

    # The marker positions in the trackable's frame of reference:
    #
    marker_positions:
      - x: 0.0714197
        y: 0.0800214
        z: 0.0622611
      - x: 0.0400755
        y: -0.0912328
        z: 0.0317064
      - x: -0.0647293
        y: -0.0879977
        z: 0.0830852
      - x: -0.0558663
        y: -0.0165446
        z: 0.053473

**Note** that each new LED position is indicated by a dash (-). The position is given in the x, y, and z coordinates of the object frame in metres.

If you would like to use your own marker positions file, place it in the `monocular_pose_tracker/marker_positions` folder and alter the launch file as explained below in the section 'Launch Files'.


#### Running the RPG Monocular Pose Estimator with a USB Camera

Ensure that you have a working USB camera.

The camera needs to be calibrated. Follow the instructions at http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration.

The pose estimator listens to the camera/image_raw topic and the  camera/camera_info topic of the camera at launch. An example of such a camera_info topic is:
    
    header: 
      seq: 4686
      stamp: 
        secs: 1378817190
        nsecs: 598124104
      frame_id: /camera
    height: 480
    width: 752
    distortion_model: plumb_bob
    D: [-0.358561237166698, 0.149312912580924, 0.000484551782515636, -0.000200189442379448, 0.0]
    K: [615.652408400557, 0.0, 362.655454167686, 0.0, 616.760184718123, 256.67210750994, 0.0, 0.0, 1.0]
    R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    P: [525.978149414062, 0.0, 357.619310580667, 0.0, 0.0, 579.796447753906, 258.377118195804, 0.0, 0.0, 0.0, 1.0, 0.0]
    binning_x: 0
    binning_y: 0
    roi: 
      x_offset: 0
      y_offset: 0
      height: 0
      width: 0
      do_rectify: False
    ---

The camera should be adjusted so that the gain and shutter/exposure time of the camera are fixed. (Use 'rqt_reconfigure'  to adjust the camera parameters while the camera is running). Ideally, the LEDs should appear very bright in the image so that they can easily be segmented from the surrounding scene by simple thresholding. See 'Parameter Settings' below on how to change the thresholding value.

#### Inputs, Outputs

##### Configuration Files

The RPG Monocular Pose Estimator requires the LED positions on the target object. These are entered in a YAML file and are loaded at runtime. See 'Setting up the Marker Positions Parameter File' above for more details.

##### Subscribed Topics

The RPG Monocular Pose Estimator subscribes to the following topics:

* camera/image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  The image form the camera. The LEDs will be detected in this image. 

* camera/camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

  The camera calibration parameters.

##### Published Topics

The RPG Monocular Pose Estimator publishes the following topics:
    
* estimated_pose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))

     The estimated pose of the target object with respect to the camera.

* image_with_detections ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  The image with the detected LEDs cirlced in red, the region of interest of the image that was processed bounded by a blue rectangle, and the orientation trivector projected onto the object.

#### Parameter Settings

The following parameters can be set dynamically during runtime. (Use 'rqt_reconfigure' to adjust the parameters).

exit(gen.generate(PACKAGE, "monocular_pose_estimator", "monocular_pose_estimator_params"))


* threshold_value (int, default: 220, min: 0, max: 255)

  This is the pixel intensity that will be used to threshold the image. All pixels with intensity values below this value will be set to zero. Pixels with intensity values equal to or higher than this value will retain their intensities.

* gaussian_sigma (double, default: 0.6, min: 0, max: 6)

  This is the standard deviation of the of the Gaussian that will be used to blur the image after being thresholded.

* min_blob_area (double, default: 10, min: 0, max: 100)

  This is the minimum blob area (in pixels squared) that will be detected as a blob/LED.

* max_blob_area (double, default: 200, min: 0, max: 1000)

  This is the maximum blob area (in pixels squared) that will be detected as a blob/LED. Blobs having an area larger than this will not be detected as LEDs.

* max_width_height_distortion (double, default: 0.5, min: 0, max: 1)

  This is a parameter related to the circular distortion of the detected blobs. It is the maximum allowable distortion of a bounding box around the detected blob calculated as the ratio of the width to the height of the bounding rectangle. Ideally the ratio of the width to the height of the bounding rectangle should be 1. 

* max_circular_distortion (double, default: 0.5, min: 0, max: 1)

  This is a parameter related to the circular distortion of the detected blobs. It is the maximum allowable distortion of a bounding box around the detected blob, calculated as the area of the blob divided by pi times half the height or half the width of the bounding rectangle. 

* back_projection_pixel_tolerance (double, default: 3, min: 0, max: 10)

  This is the tolerance (in pixels) between a back projected LED and an image detection. If a back projected LED is within this threshold, then the pose for that back projection is deemed to be a possible candidate for the pose.

* nearest_neighbour_pixel_tolerance (double, default: 5, min: 0, max: 10)

  This is the tolerance for the prediction of the correspondences between object LEDs and image detections. If the predicted position of an LED in the image is within this tolerance of the image detections, then the LED and image detection are considered to correspond to each other.

* certainty_threshold (double, default: 0.75, min: 0, max: 1)

  This is the proportion of back projected LEDs into the image that have to be within the back_projection_pixel_tolerance, for a pose to be considered valid.

* valid_correspondence_threshold (double, default: 0.7, min: 0, max: 1)

  This is the ratio of all combinations of 3 of the correspondences that yielded valid poses (i.e., were within the certainty_threshold), for a set of correspondences to be considered valid.

* roi_boarder_thickness (int, default: 10, min: 0, max: 200)

  This is the thickness of the boarder (in pixels) around the predicted area of the LEDs in the image that defines the region of interest for image processing and detection of the LEDs.

#### Launch Files

The RPG Monocular Pose Estimator needs to be launched with a launch file, since the location of the YAML configuration file containing the LED positions on the object needs to be specified. (See 'Setting up the Marker Positions Parameter File' above for further details). An example launch file is presented below.

    <launch> 
        
    	<!-- Name of the YAML file containing the marker positions -->
        <arg name="YAML_file_name" default="marker_positions"/>

    	<!-- File containing the the marker positions in the trackable's frame of reference -->
		<arg name="marker_positions_file" default="$(find monocular_pose_estimator)/marker_positions/$(arg YAML_file_name).yaml"/> 

	    <node name="monocular_pose_estimator" pkg="monocular_pose_estimator" type="monocular_pose_estimator" respawn="false" output="screen"> 
		    <rosparam command="load" file="$(arg marker_positions_file)"/>
    		<param name= "threshold_value" value = "140" />
    		<param name= "gaussian_sigma" value = "0.6" />
    		<param name= "min_blob_area" value = "10" />
    		<param name= "max_blob_area" value = "200" />
    		<param name= "max_width_height_distortion" value = "0.5" />
    		<param name= "max_circular_distortion" value = "0.5" />
    	</node>
    </launch>

The name of the YAML configuration file may be specified directly as the default value of the argument `YAML_file_name`, or may be passed to the system via the command line on launch, e.g. `roslaunch monocular_pose_estimator launch_file_name.launch YAML_file_name:=<file_name>`, where `<file_name>` is replaced with the name of the maker positions YAML file.

This example launch file also sets some of the parameters described in 'Parameter Settings' above. They needn't be set in the launch file as, they can be dynamically changed during runtime. (Use 'rqt_reconfigure' to adjust the parameters).

For more information on how to use ROS launch files, see the ROS [website](http://wiki.ros.org/ROS/Tutorials/Roslaunch%20tips%20for%20larger%20projects).

Hardware
--------

The RPG Monocular Pose Estimator makes use of infrared LEDs mounted on the target object and a camera fitted with an infrared-pass filter. The details of the LEDs and camera that were used in our evaluation of the package are outlined below. Our system will be able to work with any kind of LEDs and camera, provided that the LEDs appear bright in the image and can consequently be segmented from the image using simple thresholding operations. Also the emission pattern of the LEDs should be as wide as possible, so that they still appear bright in the image even when viewed from shallow viewing angles.

### Infrared LEDs

The infrared LEDs that were used were SMD LEDs from Harvatek HT-260IRPJ of type Harvatek HT-260IRPJ. They have a wide emission pattern and consequently can be viewed from varying view points.

#### LED Configuration

The placement of the LEDs on the target object can be arbitrary, but must be non-symmetric. The LEDs should also not lie in a plane in order to reduce the ambiguities of the pose estimation. Also the larger the volume that the LEDs fill, the more accurate the estimated pose will be. The robustness and accuracy are improved if the LEDs are visible from many view points. The RPG Monocular Pose Estimator requires a minimum of four (4) LEDs mounted and visible on the target object in order for its pose to be resolved. 

### Camera

The camera used was a [MatrixVision](http://www.matrix-vision.com/) mvBlueFOX-MLC200w monochrome camera fitted with an infrared-pass filter. Its resolution is 752x480 pixels.
