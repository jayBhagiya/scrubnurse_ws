# Scrubnurse workspace without the GPU parts (YOLO detector, Whisper voice):
# the simulated cell and the dashboard run from this image.

FROM node:20-slim AS web
WORKDIR /web
COPY src/scrubnurse_dashboard/web/package.json src/scrubnurse_dashboard/web/package-lock.json ./
RUN npm ci --no-fund --no-audit
COPY src/scrubnurse_dashboard/web/ ./
RUN npm run build

FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ws

# Dependencies first, from the package manifests only, so code changes keep this layer cached.
COPY src/scrubnurse_bringup/package.xml src/scrubnurse_bringup/
COPY src/scrubnurse_control/package.xml src/scrubnurse_control/
COPY src/scrubnurse_dashboard/package.xml src/scrubnurse_dashboard/
COPY src/scrubnurse_interfaces/package.xml src/scrubnurse_interfaces/
COPY src/scrubnurse_perception/package.xml src/scrubnurse_perception/
COPY src/scrubnurse_sim/package.xml src/scrubnurse_sim/
COPY src/scrubnurse_voice/package.xml src/scrubnurse_voice/
COPY requirements.txt .
RUN apt-get update \
 && rosdep update --rosdistro humble \
 && rosdep install --from-paths src --ignore-src -y \
 && apt-get install -y --no-install-recommends python3-pip xvfb \
 && grep -E '^(fastapi|uvicorn|websockets)==' requirements.txt | xargs pip3 install --no-cache-dir \
 && rm -rf /var/lib/apt/lists/*

COPY src src
COPY --from=web /web/dist src/scrubnurse_dashboard/web/dist
RUN source /opt/ros/humble/setup.bash && colcon build && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
