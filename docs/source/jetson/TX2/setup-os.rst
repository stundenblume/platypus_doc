==================
Setting Up the OS
==================

After flashing the Jetson image into the TX2 board, we may have to install CUDA manually. In order to do so, connect the Jetson board to the internet and check its IP address. Having the IP address, transfer the `cuda-repo-l4t-9-0-local_9.0.252-1_arm64.deb` file that is placed into the `jetpack_download` folder of the Jetpack installer to the Jetson board as:

.. code-block:: bash

    $ cd jetpack_download/
    $ scp cuda-repo-l4t-9-0-local_9.0.252-1_arm64.deb nvidia@<IP_ADDRESS>:/home/nvidia/

With the file in the `/home/nvidia/` folder in Jetson, log in the board and install CUDA with:

.. code-block:: bash

    $ sudo dpkg -i cuda-repo-l4t-9-0-local_9.0.252-1_arm64.deb
    $ sudo apt update
    $ sudo apt search cuda 

Searching by CUDA will return `cuda-toolkit-9-0` among the other options. This is the package we will install by typing:

.. code-block:: bash

    $ sudo apt-get install cuda-toolkit-9-0

After installing CUDA, we have to set the paths in `.bashrc` file, using:

.. code-block:: bash

    $ echo "" >> ~/.bashrc
    $ echo "# CUDA Toolkit" >> ~/.bashrc
    $ echo "export CUDA_HOME=/usr/local/cuda-9.0" >> ~/.bashrc
    $ echo "export PATH=$PATH:$CUDA_HOME/bin" >> ~/.bashrc
    $ echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDA_HOME/lib64" >> ~/.bashrc

Next, we have to install `Cudnn` in the system. In order to do so, transfer the `libcudnn7` package from `jetpack_download` folder and install it with:

.. code-block:: bash

    $ sudo dpkg -i libcudnn7_7.1.5.14-1+cuda9.0_arm64.deb
    $ sudo dpkg -i libcudnn7-dev_7.1.5.14-1+cuda9.0_arm64.deb

Now, we can install TensorRT that also is downloaded into `jetpack_download` folder. To do so, transfer to the board the following packages and install them as:

.. code-block:: bash

    $ sudo dpkg -i libnvinfer4_4.1.3-1+cuda9.0_arm64.deb
    $ sudo dpkg -i libnvinfer-dev_4.1.3-1+cuda9.0_arm64.deb
    $ sudo dpkg -i libnvinfer-samples_4.1.3-1+cuda9.0_arm64.deb
    $ sudo dpkg -i libgie-dev_4.1.3-1+cuda9.0_arm64.deb
    $ sudo dpkg -i tensorrt_4.0.2.0-1+cuda9.0_arm64.deb

We also install VisionWorks using:

.. code-block:: bash

    $ sudo dpkg -i libvisionworks-repo_1.6.0.500n_arm64.deb
    $ sudo dpkg -i libvisionworks-sfm-repo_0.90.3_arm64.deb
    $ sudo dpkg -i libvisionworks-tracking-repo_0.88.1_arm64.deb

Finally, we can install OpenCV package. To do so, we first have to install its dependencies with:

.. code-block:: bash

    $ sudo apt-add-repository universe
    $ sudo apt-get update
    $ sudo apt-get install cmake libavcodec-dev libavformat-dev libavutil-dev libeigen3-dev libglew-dev libgtk2.0-dev
    $ sudo apt-get install libgtk-3-dev libjasper-dev libjpeg-dev libpng12-dev libpostproc-dev libswscale-dev
    $ libtbb-dev libtiff5-dev libv4l-dev libxvidcore-dev libx264-dev qt5-default zlib1g-dev pkg-config

    # Install Python 2.7
    $ sudo apt-get install python-dev python-numpy python-py python-pytest
    # Install Python 3.5
    $ sudo apt-get install python3-dev python3-numpy python3-py python3-pytest
    # GStreamer support
    $ sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev 

    # Install OpenCV
    $ sudo dpkg -i libopencv_3.3.1_t186_arm64.deb
    $ sudo dpkg -i libopencv-python_3.3.1_t186_arm64.deb
    $ sudo dpkg -i libopencv-dev_3.3.1_t186_arm64.deb
    $ sudo dpkg -i libopencv-samples_3.3.1_t186_arm64.deb



