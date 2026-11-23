# Zotbotics · Isaac Sim 5.1.0 and Isaac ROS 4.0.0

Run both services on one native Ubuntu 24.04 x86_64 workstation with a supported NVIDIA RTX GPU. Your laptop can connect to the workstation with the Isaac Sim WebRTC Streaming Client. ROS 2 Jazzy Jalisco runs inside the containers. The host needs an NVIDIA driver, Docker Engine, Compose v2.30 or newer and the NVIDIA Container Toolkit configured for Docker.

## 1. Check the workstation

Run these on the machine that will host the containers.

```bash
nvidia-smi
docker compose version
docker run --rm --gpus all ubuntu:24.04 nvidia-smi
```

The last command should list the same GPU. NVIDIA's 5.1 requirements list a GeForce RTX 4080 with 16 GB VRAM as the minimum x86 configuration and Linux driver 580.65.06 as the tested version. For the WebRTC client, use a server GPU with NVENC support. Make room on the SSD for both large images, the ROS build and the shader cache.

[Isaac Sim 5.1 hardware requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html) · [Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/) · [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## 2. Configure and start

Extract the ZIP. Run the following from its directory containing `compose.yaml`. The simulator reads scenes from `scenes/`. The simulator runs as UID 1234. These commands give it write access to its cache and run ROS as your normal host UID/GID.

```bash
mkdir -p scenes ros_ws/src
sudo install -d -m 0755 -o 1234 -g 1234 sim-cache/kit sim-cache/kit/ov sim-cache/kit/warp sim-cache/cuda
```

After reviewing and accepting NVIDIA's Omniverse license, create the project settings. Use the default IP to connect a client on the workstation itself. If you will connect from a laptop, edit `ISAACSIM_HOST` in `.env` to the workstation's reachable LAN IPv4 address before starting.

```bash
cat > .env <<ENV
ACCEPT_EULA=Y
LOCAL_UID=$(id -u)
LOCAL_GID=$(id -g)
ISAACSIM_HOST=127.0.0.1
ROS_DOMAIN_ID=42
ENV
docker compose config --quiet
docker compose up --build -d
docker compose logs -f sim
```

Wait for `Isaac Sim Full Streaming App is loaded.` The first launch warms the shader cache. Ctrl+C exits the log viewer. Open the native Isaac Sim WebRTC Streaming Client and connect to the address in `ISAACSIM_HOST`. For LAN access, the client must be able to reach TCP 49100 and UDP 47998 on the workstation.

[Container setup and license](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html) · [WebRTC client and ports](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html)

## 3. Verify Isaac ROS and simulation time

From the project folder, check the installed NVIDIA package and its release. The prefix should be `/opt/isaac-ros-4.0.0` and the XML should show `4.0.0`.

```bash
./ros-shell ros2 pkg prefix isaac_ros_common
./ros-shell cat /opt/isaac-ros-4.0.0/share/isaac_ros_common/package.xml | grep '<version>'
```

In Isaac Sim, open a stage and choose **Tools → Robotics → ROS 2 OmniGraphs → Clock**. Accept the default graph path and press **Play**. Run the following on the host. The first command should print one simulation timestamp. The second streams timestamps until you press Ctrl+C. Pause the simulator and check that messages stop, then press Play and check that they resume.

```bash
./ros-shell ros2 topic echo /clock rosgraph_msgs/msg/Clock --once --qos-reliability best_effort
./ros-shell ros2 topic echo /clock rosgraph_msgs/msg/Clock --qos-reliability best_effort
```

Both processes use the same DDS domain and Fast DDS over UDP. If the subscriber waits indefinitely, check that the stage is playing, that the Clock graph exists, and that its ROS 2 Context reads the environment domain ID or explicitly uses the value from `.env`. Run `docker compose logs sim` for ROS bridge loading errors. The 5.1.0 image's bundled libraries live under `/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib`.

[Isaac Sim 5.1 ROS setup](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html) · [Clock graph](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_clock.html)

## 4. Add the quadruped workspace

Place the robot's USD stage and its referenced assets under `scenes/` on the host and open the stage under `/scenes` in Isaac Sim. This mount is read-only. For scene editing, change it to read/write and give UID 1234 write access to that host folder. Add the graph publishers and subscribers your controller expects, starting with the clock and joint state publisher. Use the actual topic names from `./ros-shell ros2 topic list -t`.

Put team ROS packages in `ros_ws/src/` and install their declared dependencies in the ROS Dockerfile. Build from the project directory. Each invocation of `ros-shell` sources Jazzy, the pinned Isaac ROS install and the team workspace if it has been built.

```bash
./ros-shell colcon build --symlink-install
./ros-shell
```

Set `use_sim_time: true` for nodes that should follow the simulation clock. The starter includes NVIDIA's `isaac_ros_common` and AprilTag message definitions. Choose and add any accelerated perception nodes your robot needs. Keep the selected NVIDIA repositories and their NVIDIA dependencies on original release revisions when extending the image. The release-4.0 branch and unversioned apt installs can include Update 1.

The ROS build uses the Jazzy/Noble amd64 development image corresponding to NVIDIA's original 4.0 CLI Dockerfiles. It builds the two packages from the `isaac_ros_common` Git tag `v4.0-0` and verifies commit `68d835816a906e1b5d643bffbfe72e940ae37ea9`. Both container base images are pinned by registry digest. The tag names differ between repositories, so check the release commit for each package you add.

[Isaac ROS release history](https://nvidia-isaac-ros.github.io/v/release-4.0/releases/index.html) · [Pinned source](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/tree/68d835816a906e1b5d643bffbfe72e940ae37ea9) · [NVIDIA development image](https://catalog.ngc.nvidia.com/orgs/nvidia/isaac/containers/ros/noble-ros2_jazzy_d3e84470d576702a380478a513fb3fc6-amd64/layers)

Stop both services with `docker compose down`. On subsequent starts, use `docker compose up -d`. Rebuild after changing the ROS Dockerfile.
