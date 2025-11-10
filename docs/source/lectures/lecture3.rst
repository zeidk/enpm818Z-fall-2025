L3: Visual Object Detection and Classification
===============================================

YOLO Detector Demo - Quick Start Guide

This guide walks you through setting up and running the YOLO object detection demo on the KITTI dataset.

Prerequisites
--------------

- Ubuntu 22.04 or 24.04
- ROS 2 Jazzy installed
- Python 3.10 or newer
- ~10 GB free disk space

-------------------------------

Step 1: Download the KITTI Rosbag from Google Drive
---------------------------------------------------

**Option A: Using Web Browser**

1. Go to the shared Google Drive link (provided by the instructor)
2. Download ``kitti_training.bag`` (this will be a folder/directory)
3. Move it to your desired location, e.g., ``~/datasets/``

**Option B: Using gdown (Command Line)**

.. code-block:: bash

    # Install gdown if not already installed
    pip install gdown

    # Download the bag (replace FILE_ID with actual Google Drive file ID)
    gdown --folder https://drive.google.com/drive/folders/FILE_ID

    # Or if you have a direct link
    gdown https://drive.google.com/uc?id=FILE_ID -O ~/datasets/kitti_training.bag --folder

**Option C: Create Your Own Bag from KITTI Dataset**

.. code-block:: bash

    # Download KITTI object detection dataset from:
    # http://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=2d

    # Create bag using the provided script
    python3 create_kitti_bag_jazzy.py \
        --dataset /path/to/KITTI/training \
        --output ~/datasets/kitti_training.bag \
        --frequency 10.0

**Verify the Bag**

.. code-block:: bash

    # Check bag info
    ros2 bag info ~/datasets/kitti_training.bag

    # Expected output:
    # - Duration: ~748 seconds
    # - Messages: 7481
    # - Topics: /kitti/camera/image

-------------------------------

Step 2: Clone and Setup the Package
-----------------------------------

.. code-block:: bash

    # Create workspace
    mkdir -p ~/yolo_ws/src
    cd ~/yolo_ws/src

    # Clone or copy the yolo_detector package
    cp -r /path/to/yolo_detector .

    # Or clone from repository
    # git clone <repository_url> yolo_detector

-------------------------------

Step 3: Install ROS Dependencies
--------------------------------

.. code-block:: bash

    cd ~/yolo_ws
    sudo rosdep init  # Only if not done before
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

This installs:
- ``ros-jazzy-cv-bridge``
- ``ros-jazzy-vision-msgs``
- ``ros-jazzy-sensor-msgs``
- ``ros-jazzy-std-msgs``

-------------------------------

Step 4: Install Python Dependencies
-----------------------------------

**Fix NumPy Compatibility Issue**

.. code-block:: bash

    pip uninstall numpy -y
    pip install "numpy<2"
    python3 -c "import numpy; print(numpy.__version__)"

**Install YOLO and Other Dependencies**

.. code-block:: bash

    cd ~/yolo_ws/src/yolo_detector
    pip install -r requirements.txt

**Verify Installation**

.. code-block:: bash

    python3 -c "from cv_bridge import CvBridge; print('✓ cv_bridge works')"
    python3 -c "from ultralytics import YOLO; print('✓ ultralytics works')"
    python3 -c "import cv2; print('✓ opencv works')"

-------------------------------

Step 5: Download YOLO Weights
-----------------------------

Note: They are already in the package's config folder.

.. code-block:: bash

    python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
    python3 -c "from ultralytics import YOLO; YOLO('yolov8s.pt')"
    python3 -c "from ultralytics import YOLO; YOLO('yolov8m.pt')"

**Available Models**

+---------------+--------+-----------+------------+-------------------------+
| Model         | Size   | Speed     | Accuracy   | Use Case                |
+===============+========+===========+============+=========================+
| yolov8n.pt    | 6 MB   | Fastest   | Good       | Real-time, embedded     |
| yolov8s.pt    | 22 MB  | Fast      | Better     | Balanced                |
| yolov8m.pt    | 52 MB  | Medium    | Excellent  | Accuracy-focused        |
| yolov8l.pt    | 87 MB  | Slow      | Great      | Best accuracy           |
| yolov8x.pt    | 136 MB | Slowest   | Best       | Maximum accuracy        |
+---------------+--------+-----------+------------+-------------------------+

-------------------------------

Step 6: Configure Parameters (Optional)
---------------------------------------

Edit parameter file:

.. code-block:: bash

    nano ~/yolo_ws/src/yolo_detector/config/yolo_params.yaml

Example:

.. code-block:: yaml

    yolo_detector:
      ros__parameters:
        model_path: "yolov8n.pt"
        conf_threshold: 0.5
        iou_threshold: 0.45
        image_topic: "/kitti/camera/image"
        max_det: 300
        device: ""

**Pre-configured Files:**

- ``yolo_params.yaml`` — Default
- ``yolo_params_sensitive.yaml`` — More detections
- ``yolo_params_conservative.yaml`` — Fewer detections

-------------------------------

Step 7: Build the Package
-------------------------

.. code-block:: bash

    cd ~/yolo_ws
    colcon build --packages-select yolo_detector
    source install/setup.bash
    echo "source ~/yolo_ws/install/setup.bash" >> ~/.bashrc

Verify build:

.. code-block:: bash

    ros2 pkg list | grep yolo_detector
    ros2 pkg executables yolo_detector

-------------------------------

Step 8: Launch the Demo
-----------------------

**Basic Launch**

.. code-block:: bash

    ros2 launch yolo_detector yolo_detector.launch.py \
        bag_file:=~/datasets/kitti_training.bag \
        play_rate:=0.1



**Playback Speed Options**

- ``play_rate:=1.0`` — Normal speed
- ``play_rate:=0.5`` — Recommended for viewing
- ``play_rate:=0.1`` — Good for analysis

**Manual Launch (Separate Components)**

.. code-block:: bash

    # Detector node
    ros2 run yolo_detector yolo_detector_node --ros-args --params-file ...
    # Bag player
    ros2 bag play ~/datasets/kitti_training.bag --loop --rate 0.5
    # RViz
    rviz2 -d $(ros2 pkg prefix yolo_detector)/share/yolo_detector/config/demo.rviz

-------------------------------

Step 9: Visualize with Foxglove Studio
--------------------------------------

**Install Foxglove Bridge**

.. code-block:: bash

    sudo apt install ros-jazzy-foxglove-bridge
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml

**Connect to ROS 2**

- Open Foxglove Studio (Desktop or Web)
- Connect to ``ws://localhost:8765``

**Load Demo Layout**

- Import ``demo.json`` from ``config/``

**Panels Overview**

- Raw Image → ``/kitti/camera/image``
- Detections Image → ``/detections/image``
- Detection Count → ``/detections/count``
- Inference Time → ``/detections/inference_time``
- FPS → ``/detections/fps``
- Stats (JSON) → ``/detections/stats``

-------------------------------

Expected Output
---------------

**In RViz:**

- Two image panels showing raw and detected images.

**In Foxglove:**

- Two image panels showing raw and detected images.
- Real-time plots (detections, FPS, inference time).

**In Terminal:**

.. code-block:: text

    [INFO] [yolo_detector]: Frame 30: 5 objects | Inference: 45.2ms | Avg: 47.3ms (21.1 FPS)

-------------------------------

Monitoring and Debugging
------------------------

.. code-block:: bash

    ros2 topic list
    ros2 topic hz /kitti/camera/image
    ros2 topic hz /detections
    ros2 param list /yolo_detector
    ros2 param get /yolo_detector conf_threshold
    ros2 param set /yolo_detector conf_threshold 0.3

-------------------------------

Troubleshooting
---------------

**NumPy Version Error**

.. code-block:: bash

    pip uninstall numpy -y
    pip install "numpy<2"

**Package Not Found**

.. code-block:: bash

    colcon build --packages-select yolo_detector
    source install/setup.bash

**YOLO Model Not Found**

.. code-block:: bash

    python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"


Stopping the Demo
-----------------

.. code-block:: bash

    Ctrl+C
    pkill -9 ros2
    pkill -9 rviz2

