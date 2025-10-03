#!/bin/bash

# apt-get update -y
# apt-get upgrade -y

apt-get install -y gcc libffi-dev
apt-get install -y libcairo2 libcairo2-dev
apt-get install -y build-essential cmake git unzip pkg-config
apt-get install -y libgirepository1.0-dev gobject-introspection
# apt-get install -y libjpeg-dev libpng-dev
# apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
# apt-get install -y libxvidcore-dev libx264-dev
# apt-get install -y libtbb-dev libdc1394-22-dev
# apt-get install -y libv4l-dev v4l-utils
# apt-get install -y libopenblas-dev libatlas-base-dev libblas-dev
# apt-get install -y liblapack-dev gfortran libhdf5-dev
# apt-get install -y libprotobuf-dev libgoogle-glog-dev libgflags-dev
# apt-get install -y protobuf-compiler

apt-get install -y gstreamer1.0* libgstrtspserver-1.0-dev
apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev
apt-get install -y gir1.2-glib-2.0
# apt-get install -y gir1.2-gst-rtsp-server-1.0

apt-get install -y libcamera-apps libcamera-dev
apt-get install -y python3-dev python3-numpy python3-pip python3-opencv
