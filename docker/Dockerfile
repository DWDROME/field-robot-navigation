# syntax=docker/dockerfile:1
ARG ROS_IMAGE=ros:jazzy-ros-base-noble@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f

FROM ${ROS_IMAGE} AS build-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG BUILD_JOBS=2
ENV DEBIAN_FRONTEND=noninteractive
COPY config/build.packages /build/packages/
RUN test "$(dpkg --print-architecture)" = amd64 \
    && mapfile -t packages < <(grep -Ev '^[[:space:]]*(#|$)' /build/packages/build.packages) \
    && apt-get -o Acquire::Retries=3 update \
    && for attempt in 1 2 3; do \
        apt-get -o Acquire::Retries=3 install -y --no-install-recommends "${packages[@]}" && break; \
        if [ "$attempt" = 3 ]; then exit 1; fi; \
    done \
    && rm -rf /var/lib/apt/lists/*
COPY --from=dependencies . /build/third_party/
COPY --from=build_tools prepare-dependencies.py /build/tools/prepare-dependencies.py

FROM build-base AS sdk-builder
RUN python3 /build/tools/prepare-dependencies.py ros2 /build/deps livox_sdk2 \
    && cmake -S /build/deps/livox_sdk2 -B /build/sdk -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /build/sdk --parallel ${BUILD_JOBS} && cmake --install /build/sdk

FROM build-base AS gtsam-builder
RUN python3 /build/tools/prepare-dependencies.py ros2 /build/deps gtsam \
    && cmake -S /build/deps/gtsam -B /build/gtsam \
        -DCMAKE_INSTALL_PREFIX=/opt/greenhouse_gtsam -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=OFF \
    && cmake --build /build/gtsam --parallel ${BUILD_JOBS} && cmake --install /build/gtsam

FROM build-base AS far-builder
RUN python3 /build/tools/prepare-dependencies.py ros2 /build/deps autonomy_stack_go2 \
    && source /opt/ros/jazzy/setup.bash && cd /build/deps/autonomy_stack_go2 \
    && MAKEFLAGS=-j${BUILD_JOBS} colcon build --executor sequential --merge-install \
        --base-paths src/base_autonomy/terrain_analysis_ext src/route_planner \
        --packages-select visibility_graph_msg terrain_analysis_ext far_planner graph_decoder boundary_handler \
        --install-base /opt/greenhouse_far --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM build-base AS robot-builder
COPY --from=sdk-builder /usr/local/ /usr/local/
COPY --from=gtsam-builder /opt/greenhouse_gtsam /opt/greenhouse_gtsam
COPY --from=far-builder /opt/greenhouse_far /opt/greenhouse_far
ENV CMAKE_PREFIX_PATH=/opt/greenhouse_gtsam
RUN python3 /build/tools/prepare-dependencies.py ros2 /build/robot/external fast_lio serial
COPY --from=robot_source drivers /build/robot/src/drivers
COPY --from=robot_source perception /build/robot/src/perception
COPY --from=robot_source navigation /build/robot/src/navigation
COPY --from=robot_source control /build/robot/src/control
COPY --from=robot_source bringup /build/robot/src/bringup
COPY --from=robot_source localization/fast_lio_super /build/robot/src/localization/fast_lio_super
COPY --from=robot_source localization/fast_lio_super_ros2 /build/robot/src/localization/fast_lio_super_ros2
COPY --from=robot_source localization/slam_octvox /build/robot/src/localization/slam_octvox
RUN cp /build/robot/src/drivers/livox_ros_driver2/package_ROS2.xml /build/robot/src/drivers/livox_ros_driver2/package.xml \
    && printf '%s\n' /opt/greenhouse_gtsam/lib > /etc/ld.so.conf.d/greenhouse-gtsam.conf && ldconfig \
    && source /opt/ros/jazzy/setup.bash && source /opt/greenhouse_far/local_setup.bash \
    && cd /build/robot \
    && MAKEFLAGS=-j${BUILD_JOBS} colcon build --executor sequential --merge-install \
        --base-paths src external --packages-ignore fast_lio_super slam_octvox greenhouse_sim_air \
        --install-base /opt/greenhouse \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
          -DEigen3_DIR=/usr/share/eigen3/cmake -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy

FROM ${ROS_IMAGE} AS discovery-runtime
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get -o Acquire::Retries=3 update \
    && apt-get -o Acquire::Retries=3 install -y --no-install-recommends iproute2 ros-jazzy-fastrtps \
    && rm -rf /var/lib/apt/lists/*
COPY fastdds/server.xml /etc/greenhouse/fastdds/server.xml
HEALTHCHECK --interval=10s --timeout=3s --start-period=5s --retries=3 \
    CMD ss -H -lun | grep -Eq '[:.]11811[[:space:]]' || exit 1
CMD ["fastdds", "discovery", "--server-id", "0", "--udp-address", "0.0.0.0", "--udp-port", "11811", "--xml-file", "/etc/greenhouse/fastdds/server.xml"]

FROM ${ROS_IMAGE} AS production-runtime
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
COPY config/runtime.packages /build/packages/
RUN test "$(dpkg --print-architecture)" = amd64 \
    && mapfile -t packages < <(grep -Ev '^[[:space:]]*(#|$)' /build/packages/runtime.packages) \
    && apt-get -o Acquire::Retries=3 update \
    && for attempt in 1 2 3; do \
        apt-get -o Acquire::Retries=3 install -y --no-install-recommends "${packages[@]}" && break; \
        if [ "$attempt" = 3 ]; then exit 1; fi; \
    done \
    && rm -rf /var/lib/apt/lists/* /build
COPY --from=sdk-builder /usr/local/lib/liblivox_lidar_sdk_shared.so* /usr/local/lib/
COPY --from=gtsam-builder /opt/greenhouse_gtsam /opt/greenhouse_gtsam
COPY --from=far-builder /opt/greenhouse_far /opt/greenhouse_far
COPY --from=robot-builder /opt/greenhouse /opt/greenhouse
COPY fastdds/participant.xml /etc/greenhouse/fastdds/participant.xml
COPY --chmod=755 scripts/ros-entrypoint.sh /ros_entrypoint.sh
RUN printf '%s\n' /opt/greenhouse_gtsam/lib > /etc/ld.so.conf.d/greenhouse-gtsam.conf && ldconfig
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "greenhouse_nav2_bringup", "closed_loop.launch.py", "deployment:=/config/deployment.yaml"]
