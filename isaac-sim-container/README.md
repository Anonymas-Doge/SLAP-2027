# Zotbotics quadruped · Isaac Sim 5.1.0 and Isaac ROS 4.0.0

| Machine | Software and role | Command after setup |
|---|---|---|
| Ubuntu 24.04 x86_64 RTX workstation | Docker runs Isaac Sim and its ROS 2 Jazzy bridge. | `docker compose up -d sim` |
| Jetson AGX Thor with JetPack 7.0 | Docker runs the ARM64 Isaac ROS development environment and the team’s nodes. | `./thor up -d` |
| Viewer on the workstation or on a laptop, including Windows | The Isaac Sim WebRTC Streaming Client displays the simulator and accepts keyboard and mouse input. | Connect to the workstation IP. |

## Before you start

Copy the ZIP to both Linux machines. Extract it separately on each to preserve executable permissions. Run the commands below from the extracted project directory on the indicated computer.

```bash
unzip Zotbotics_Isaac_Sim_5_1_0_ROS_4_0_0.zip
cd zotbotics-isaac-jazzy
```

Use Docker Engine with the Compose v2 plugin, NVIDIA Container Toolkit and a user account with Docker access. The first run downloads the images from NVIDIA and builds the ROS packages from GitHub. Allow room on the SSD for the images and build cache. ROS 2 Jazzy runs inside the containers. Connect the two Linux machines to the same LAN with peer-to-peer traffic and UDP multicast enabled. A wired switch works well for the first test.

## 1. Set up the RTX workstation

Run these on the machine that will host the containers.

```bash
nvidia-smi
docker compose version
docker run --rm --gpus all ubuntu:24.04 nvidia-smi
```

The last command should list the same GPU. NVIDIA's 5.1 requirements list a GeForce RTX 4080 with 16 GB VRAM as the minimum x86 configuration and Linux driver 580.65.06 as the tested version. For the WebRTC client, use a server GPU with NVENC support. Make room on the SSD for both large images, the ROS build and the shader cache.

[Isaac Sim 5.1 hardware requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html) · [Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/) · [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

The simulator reads scenes from `scenes/`. The simulator runs as UID 1234. These commands give it write access to its cache and run ROS as your normal host UID/GID.

```bash
mkdir -p scenes ros_ws/src
sudo install -d -m 0755 -o 1234 -g 1234 sim-cache/kit sim-cache/kit/ov sim-cache/kit/warp sim-cache/cuda
```

After reviewing and accepting NVIDIA's Omniverse license, create the project settings on first use. If you already have a project `.env`, edit its values in place. Use the default IP to connect a client on the workstation itself. If you will connect from a laptop, edit `ISAACSIM_HOST` in `.env` to the workstation's reachable LAN IPv4 address before starting.

```bash
cat > .env <<ENV
ACCEPT_EULA=Y
LOCAL_UID=$(id -u)
LOCAL_GID=$(id -g)
ISAACSIM_HOST=127.0.0.1
ROS_DOMAIN_ID=42
ENV
docker compose config --quiet
docker compose up -d sim
docker compose logs -f sim
```

Wait for `Isaac Sim Full Streaming App is loaded.` The first launch warms the shader cache. Ctrl+C exits the log viewer. Open the native Isaac Sim WebRTC Streaming Client and connect to the address in `ISAACSIM_HOST`. For LAN access, the client must be able to reach TCP 49100 and UDP 47998 on the workstation.

[Container setup and license](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_container.html) · [WebRTC client and ports](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html)

[Download the 5.1-compatible WebRTC client](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html). The viewer address controls streaming. ROS traffic between the workstation and the Thor uses DDS independently.

## 2. Set up Isaac ROS on the Jetson

Target hardware is the AGX Thor developer kit (T5000) running JetPack 7.0, Ubuntu 24.04 and Jetson Linux R38.2 or R38.2.1. NVIDIA lists a 128 GB or larger NVMe SSD. Check the installed image, disk space and Docker runtime before pulling the large ARM64 development image.

```bash
cat /proc/device-tree/model; echo
head -1 /etc/nv_tegra_release
df -h .
docker compose version
docker info --format '{{json .Runtimes}}'
```

Use Docker Engine with the Compose v2 plugin and NVIDIA Container Toolkit. The Docker runtimes output should include `nvidia`. If it is missing, follow the [Thor Docker setup](https://docs.nvidia.com/jetson/agx-thor-devkit/user-guide/latest/setup_docker.html). Use a login with Docker access so team workspace files are owned by your account.

```bash
chmod +x thor ros-shell
./thor up --build -d
./thor ps
./ros-shell ros2 pkg prefix isaac_ros_common
./ros-shell cat /opt/isaac-ros-4.0.0/share/isaac_ros_common/package.xml | grep '<version>'
./ros-shell python3 /opt/check-cuda.py
```

Expect `/opt/isaac-ros-4.0.0`, package version `4.0.0` and `CUDA allocation and memset passed.` The CUDA check allocates GPU memory, writes to it and synchronizes inside the container. The starter builds NVIDIA's common utilities and AprilTag message definitions from the original 4.0.0 source revision. Add the packages required by your quadruped to the ROS image or to `ros_ws/src`. Build team packages on the Thor with `./ros-shell colcon build --symlink-install`. Copy source code between the machines and build separately for each CPU architecture.

[Isaac ROS release notes](https://nvidia-isaac-ros.github.io/v/release-4.0/releases/index.html) · [JetPack release mapping](https://developer.nvidia.com/embedded/jetpack-archive)

## 3. Verify the workstation-to-Jetson connection

Put both machines on a LAN that allows peer traffic and UDP multicast discovery. Both Compose configurations mount the same Fast DDS UDP profile and default to domain 42. For a different domain, edit `ROS_DOMAIN_ID` in the workstation’s `.env` and create or edit a `.env` with the same value on the Thor. Reapply with `docker compose up -d sim` on the workstation and `./thor up -d` on the Jetson.

In Isaac Sim, choose **Tools → Robotics → ROS 2 OmniGraphs → Clock**, confirm the graph, and press **Play**. On the Jetson run:

```bash
./ros-shell ros2 topic echo /clock rosgraph_msgs/msg/Clock --once --qos-reliability best_effort
```

A timestamp confirms delivery from the workstation to the Jetson. If the command waits, check the simulator is playing and its Clock graph's ROS 2 Context uses domain 42. Then check DDS discovery across the LAN. Run `./ros-shell ros2 multicast receive` on the Jetson and `ros2 multicast send` in a sourced ROS terminal on the workstation, if available. Campus or guest Wi-Fi may isolate clients. A wired switch is a straightforward way to test the two hosts together. For a quadruped stage with a joint-state publisher, check its actual topic name with `./ros-shell ros2 topic list -t`.

[Isaac Sim 5.1 Clock graph](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_clock.html) · [ROS bridge and DDS](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)

## 4. Add the quadruped and team packages

Place the robot's USD stage and its referenced assets under `scenes/` on the workstation and open the stage under `/scenes` in Isaac Sim. This mount is read-only. For scene editing, change it to read/write and give UID 1234 write access to that host folder. Add the graph publishers and subscribers your controller expects, starting with the clock and joint state publisher. Use the actual topic names from `./ros-shell ros2 topic list -t`.

On the Thor, put team ROS packages in `ros_ws/src/` and install their declared dependencies in the ROS Dockerfile. Build from the project directory. Each invocation of `ros-shell` sources Jazzy, the pinned Isaac ROS install and the team workspace if it has been built.

```bash
./ros-shell colcon build --symlink-install
./ros-shell
```

Set `use_sim_time: true` for nodes that should follow the simulation clock. The starter includes NVIDIA's `isaac_ros_common` and AprilTag message definitions. Choose and add any accelerated perception nodes your robot needs. Keep the selected NVIDIA repositories and their NVIDIA dependencies on original release revisions when extending the image. The release-4.0 branch and unversioned apt installs can include Update 1.

## Workstation-only development

When the Thor is unavailable, the same workstation can run both containers. Use the workstation prerequisites, cache setup and `.env` from step 1, including your UID/GID. From the workstation project directory run:

```bash
docker compose up --build -d sim ros
./ros-shell ros2 pkg prefix isaac_ros_common
./ros-shell ros2 topic echo /clock rosgraph_msgs/msg/Clock --once --qos-reliability best_effort
```

Use the same playing Clock graph as in step 3. Place team source in the workstation’s `ros_ws/src/` for this mode. Build separately for AMD64 and ARM64. To return ROS processing to the Thor, stop the workstation ROS service with `docker compose stop ros` before launching duplicate nodes on the Jetson.

## Daily commands

| Task | RTX workstation | Jetson AGX Thor |
|---|---|---|
| Start | `docker compose up -d sim` | `./thor up -d` |
| Inspect status | `docker compose ps` | `./thor ps` |
| Read logs | `docker compose logs -f sim` | `./thor logs ros` |
| Open a sourced ROS shell | `./ros-shell` if the workstation ROS service is running | `./ros-shell` |
| Rebuild after a ROS Dockerfile change | `docker compose up --build -d ros` in workstation-only mode | `./thor up --build -d` |
| Stop the project | `docker compose down` | `./thor down` |

## Troubleshooting

- **Workstation GPU missing in Docker.** Run the host and container GPU checks in step 1. If only the container check fails, check the NVIDIA Container Toolkit setup for Docker.
- **The viewer cannot connect.** Check the simulator readiness message and `ISAACSIM_HOST` in the workstation’s `.env`. After editing it, run `docker compose up -d sim` to apply the change. Check reachability of the two streaming ports from the viewer.
- **The Jetson helper says “permission denied.”** From the project directory run `chmod +x thor ros-shell`. For a Docker socket error, check account access with `docker info`.
- **The clock is visible but echo waits.** Inspect the publisher count and QoS using `./ros-shell ros2 topic info /clock --verbose`, check the timeline is playing and use the best-effort subscription in step 3.
- **The quadruped scene opens with missing meshes.** Preserve the original relative paths between the USD stage and referenced assets under the workstation’s `scenes/` directory.

The Jetson and workstation use separately pinned ARM64 and AMD64 base images. The shared Dockerfile checks out the original Isaac ROS common release tag `v4.0-0` and verifies commit `68d835816a906e1b5d643bffbfe72e940ae37ea9`. The Jetson build targets CUDA architecture 110.

[Pinned Isaac ROS source](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/tree/68d835816a906e1b5d643bffbfe72e940ae37ea9)
