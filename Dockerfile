ARG base_image="robotnik/ros"
ARG ros_distro="humble"
ARG version="0.5.0"
FROM ${base_image}:${ros_distro}-builder-${version} as builder

# Copy source code
COPY --chown=${USER_NAME}:${USER_NAME} . ${USER_WORKSPACE}/src/vectornav 

# Clone dependencies
RUN vcs import --input src/vectornav/common.repos ./src

# Generate debian packages
RUN generate_debs.sh

FROM ${base_image}:${ros_distro}-base-${version}

# Mount debian packages from previous stage and install all of them
USER root
RUN --mount=type=bind,from=builder,source=${USER_WORKSPACE}/debs,target=/tmp/debs \
    apt update && \
    apt-get install -y --no-install-recommends \
        /tmp/debs/*.deb \
    && rm -rf /var/lib/apt/lists/*

USER $USER_NAME


# ENIRONMENT VARIABLES

ENV STARTUP_TYPE "launch"
# package to launch
ENV ROS_BU_PKG "vectornav"
# script of program to launch with all its arguments
ENV ROS_BU_LAUNCH "vectornav.launch.py"

### Required nodes to startup
#### if true check if the NODES_TO_CHECK nodes are started up before starting the program
ENV CHECK_NODES "false"
#### space separted node list with full namespace
ENV NODES_TO_CHECK ""
ENV HEALTHCHECK_NODES ""
