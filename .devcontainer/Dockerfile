# FR3_PC
FROM althack/ros2:humble-cuda-dev

# Build arg passed from devcontainer.json
ARG WORKSPACE=/ros2_ws
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update \
   && apt-get -y install --no-install-recommends\
    build-essential \
    cmake \
    git \
    wget \
    libeigen3-dev \
    libpoco-dev \
    libfmt-dev \
    pybind11-dev

# ENV VERSION=0.19.0

# RUN wget https://github.com/frankarobotics/libfranka/releases/download/${VERSION}/libfranka_${VERSION}_jammy_amd64.deb
# RUN wget https://github.com/frankarobotics/libfranka/releases/download/${VERSION}/libfranka_${VERSION}_jammy_amd64.deb.sha256
# RUN sha256sum -c libfranka_${VERSION}_jammy_amd64.deb.sha256
# RUN sudo dpkg -i libfranka_${VERSION}_jammy_amd64.deb

# Install required ROS packages (C+P from Adam's docker, with ros version freeze)
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends\
   python3-tk \
   ros-humble-rviz2 \
   ros-humble-control-toolbox \
   ros-humble-cv-bridge \
#  ros-humble-moveit-py \  doesn't exist
   ros-humble-moveit-configs-utils \
   ros-humble-moveit-planners \
   ros-humble-moveit-setup-assistant \
   ros-humble-moveit-servo \
   ros-humble-moveit-simple-controller-manager \
   ros-humble-moveit-resources-panda-moveit-config \
   ros-humble-ros2-control \
   ros-humble-joint-state-broadcaster \
   ros-humble-joint-state-publisher \
   ros-humble-joint-trajectory-controller \
   ros-humble-realsense2-camera \
   ros-humble-pinocchio \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Install required python modules (C+P from Adam's docker)
RUN pip install numpy==1.26.4 \
 && pip install opencv-python \
 && pip install pyassimp \
 && pip install scipy \
 && pip install transitions \
 && pip install setuptools==58.2.0 \
 && pip install pymodbus \
 && pip install ipykernel \
 && pip install icecream \
 && pip install typing_extensions==4.4.0\
 && pip install FreeSimpleGUI \
 && pip install roboticstoolbox-python 
   
   #torch torchvision timm scikit-learn


# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

################
# uncomment below for nvidia support
# Expose the nvidia driver to allow opengl 
# Dependencies for glvnd and X11.
################
RUN apt-get update \
 && apt-get install -y -qq --no-install-recommends \
  libglvnd0 \
  libgl1 \
  libglx0 \
  libegl1 \
  libxext6 \
  libx11-6 \
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

ENV QT_X11_NO_MITSHM=1

# installing missing dependancies
RUN apt-get update \
   && apt-get -y install --no-install-recommends\
   ros-humble-ros-testing \
   ros-humble-example-interfaces \
   ros-humble-moveit-resources-fanuc-moveit-config \
   ros-humble-moveit-resources-fanuc-description \
   ros-humble-teleop-twist-joy \
   ros-humble-teleop-twist-keyboard \
   ros-humble-realsense2-description \
   ros-humble-sick-safetyscanners2 \
   ros-humble-ros-gz \
   ros-humble-sdformat-urdf \
   ros-humble-gz-ros2-control \
   ros-humble-moveit-resources-pr2-description \
   ros-humble-demo-nodes-cpp \
   # ros-humble-warehouse-ros-mongo \
   freeglut3-dev \
   ros-humble-hardware-interface-testing \
   libqt5opengl5-dev \
   ros-humble-position-controllers \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
   && apt-get -y install --no-install-recommends\
   libgtest-dev \
   libgmock-dev \
   ros-humble-py-binding-tools\
   ros-humble-rviz-visual-tools\
   ros-humble-graph-msgs\
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*

# reinstall packages needed to run Adam's old code
RUN pip install numpy==1.26.4 
RUN pip install scipy==1.11.4



# rosdep init only - do NOT rosdep update here
# rosdep update is handled in postCreateCommand so it always uses
# a fresh index rather than one baked into the image at build time
RUN rosdep init || true

WORKDIR ${WORKSPACE}

COPY entrypoint.sh /entrypoint.sh
# COPY .devcontainer/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]