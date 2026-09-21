ARG PYLON_CORE_IMAGE=pylon-spaceros-core:jazzy-2026.07.0
FROM ${PYLON_CORE_IMAGE}
SHELL ["/bin/bash", "-eo", "pipefail", "-c"]
RUN mkdir -p /home/spaceros-user/demo_ws
COPY --chown=spaceros-user:spaceros-user pylon_demo_reusable /home/spaceros-user/demo_ws/src/pylon_demo_reusable
WORKDIR /home/spaceros-user/demo_ws
RUN source /opt/ros/spaceros/setup.bash && \
    source /home/spaceros-user/pylon_ws/install/local_setup.bash && \
    colcon build --packages-select pylon_demo_reusable --executor sequential
COPY scripts/spaceros-entrypoint.sh /pylon-demo-entrypoint.sh
ENTRYPOINT ["/pylon-demo-entrypoint.sh"]
CMD ["ros2", "launch", "pylon_demo_reusable", "demo.launch.py"]
