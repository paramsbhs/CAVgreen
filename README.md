# CAVgreen
Simulation Repository for CAV Team Green

## Quick Start

### Prerequisites

- Ubuntu 20.04 or 24.04
- Docker

### 1. Install Docker

**Ubuntu 20.04/24.04:**

```bash
# Remove old versions
sudo apt remove docker.io docker-compose -y

# Install Docker from official repo
sudo apt update
sudo apt install ca-certificates curl -y
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y
```

**Enable Docker:**

```bash
sudo systemctl start docker
sudo systemctl enable docker
sudo usermod -aG docker $USER
newgrp docker
```

**Verify:**

```bash
docker --version
docker compose version
```

### 2. Clone Repository

```bash
git clone --recurse-submodules https://github.com/paramsbhs/CAVgreen.git
cd CAVgreen
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

### 3. Build Docker Image

```bash
docker compose build
```

### 4. Start Containers

```bash
docker compose up -d
```

Check they're running:

```bash
docker compose ps
```

Both `sim` and `novnc` should show "Up".

### 5. Build ROS Packages

```bash
docker compose exec sim bash
source /opt/ros/foxy/setup.bash
cd /sim_ws
colcon build
source install/setup.bash
```

---

## Running the Simulator

### View the Simulator

Open your browser:

```
http://localhost:8080/vnc.html
```

### Basic Simulator Launch

**Terminal 1 - Simulator:**

```bash
docker compose exec sim bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Terminal 2 - Keyboard Control:**

```bash
docker compose exec sim bash
source /opt/ros/foxy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` = forward
- `k` = stop
- `j` = left
- `l` = right
- `u` / `o` = turn while moving


## Safety Node

Automatically stops the car before collisions using Instantaneous Time to Collision (iTTC).

### Run Safety Node

**Terminal 1 - Simulator:**

```bash
docker compose exec sim bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Terminal 2 - Safety Node:**

```bash
docker compose exec sim bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/setup.bash
ros2 launch safety_node safety_node_launch.py
```

**Terminal 3 - Keyboard Control:**

```bash
docker compose exec sim bash
source /opt/ros/foxy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### Parameters

- `ttc_threshold`: Time to collision threshold in seconds (default: 0.5)

## Lab 3: Wall Following

### Run Wall Follow Node

**Terminal 1 - Simulator:**

```bash
docker compose exec sim bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Terminal 2 - Wall Follow:**

```bash
docker compose exec sim bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/setup.bash
ros2 launch wall_follow wall_follow_launch.py
```

### Parameters

Adjust in `src/wall_follow/launch/wall_follow_launch.py`:

- `kp`: Proportional gain
- `ki`: Integral gain
- `kd`: Derivative gain
- `desired_distance`: Target distance from wall (meters)
- `lookahead_L`: Lookahead distance for prediction (meters)
- `theta`: Angle between laser beams (degrees)


## Troubleshooting

### Docker won't start

```bash
sudo systemctl start docker
sudo systemctl status docker
```

### Can't enter container

```bash
newgrp docker
```

Or use sudo:

```bash
sudo docker compose exec sim bash
```

### GUI not showing in browser

Check containers are running:

```bash
docker compose ps
```

Try: `http://localhost:8080/vnc.html`

Restart containers:

```bash
docker compose down
docker compose up -d
```

### Build errors

```bash
docker compose exec sim bash
cd /sim_ws
rm -rf build/ install/ log/
source /opt/ros/foxy/setup.bash
colcon build
```

### ROS 2 commands not found

Always source first:

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/setup.bash
```

### Launch file not found

Ensure CMakeLists.txt has the install rule:

```cmake
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

Then clean rebuild:

```bash
cd /sim_ws
rm -rf build/<package> install/<package>
colcon build --packages-select <package>
```

---

## ROS 2 Topics

### Published by Simulator

- `/scan` - LaserScan messages (LiDAR data)
- `/ego_racecar/odom` - Odometry (position, velocity)

### Subscribe to Control Car

- `/drive` - AckermannDriveStamped (speed, steering angle)

### Useful Commands

```bash
# List all topics
ros2 topic list

# See topic data
ros2 topic echo /scan

# See topic info
ros2 topic info /scan

# Publish manual drive command
ros2 topic pub /drive ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 2.0, steering_angle: 0.0}}"
```

## Resources

- [F1TENTH Documentation](https://f1tenth.org/)
- [ROS 2 Foxy Documentation](https://docs.ros.org/en/foxy/)
- [F1TENTH Gym ROS](https://github.com/f1tenth/f1tenth_gym_ros)
