# Plan Move Execute: xArm7 Local Planner Evaluation

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)
![Python](https://img.shields.io/badge/Python-3.10-blue.svg)
![Simulator](https://img.shields.io/badge/Simulator-PyBullet-orange.svg)

**Plan Move Execute** is a ROS 2-based framework designed for evaluating various local motion planning algorithms on the UFACTORY xArm7 robotic manipulator. The project integrates multiple bio-inspired and artificial potential field (APF) based planning strategies within a simulated environment to benchmark their performance across different scenarios.

## Repository Structure

The workspace consists of the following core ROS 2 packages:

* **`xarm_local_planner`**: Contains the implementations of the local planning algorithms (Beetle, PSO, Vortex, Virtual Hills, Virtual Obstacle, Hybrid, etc.) and scenario generation logic.
* **`xarm_planner_evaluation`**: The central evaluation framework. Orchestrates the execution of planners, records metrics (success rate, path length, execution time), and saves the results.
* **`xarm_sim_api`**: A Python-based API interfacing with PyBullet to simulate the xArm7, environments, and collision detection.
* **`xarm_local_msgs`**: Custom ROS 2 message (`.msg`) and service (`.srv`) definitions required for communication between the planner, simulator, and evaluator.
* **`BashScripts/`**: A suite of utility scripts for managing the Docker environment and workspace builds.

### External Dependencies
* **`xArm-Developer/xarm_ros2`**: This project integrates the official [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) repository. Please note that this dependency is used **solely as the hardware driver** to facilitate communication and trajectory execution on the physical, real-world xArm7 manipulator, rather than for the core simulation logic.
---

## Getting Started

This project is fully containerized using Docker to ensure dependency consistency.

### 1. Prerequisites
* [Docker](https://docs.docker.com/get-docker/) installed.

### 2. Launching Docker

Navigate to the root of the repository and use the provided bash scripts to manage the container lifecycle.

**Step A: Build the Docker Image**
```bash
./BashScripts/docker-build.sh
```
*This command builds the Docker image using the provided `Dockerfile`.*

**Step B: Run the Docker Container**
```bash
./BashScripts/docker-run.sh
```
*This launches the container and mounts the local repository directory as a volume inside the container.*

**Step C: Connect Additional Terminals (Optional)**
If you want to connect to the existing container:
```bash
./BashScripts/docker-connect.sh
```

### 3. Building the Workspace

Once inside the Docker container, compile the ROS 2 workspace using the provided script:

```bash
./BashScripts/colcon-build.sh
```

After building, don't forget to source the workspace:
```bash
source install/setup.bash
```

---

## Launching the Evaluation

The evaluation framework automatically cycles a specific planner through a set of predefined scenarios, collects metrics, and outputs a JSON results file.

To launch an evaluation, use the provided ROS 2 launch file. The system consists of 4 main nodes (Physics Simulator, Planning Scene, the Planner, and the Evaluator) which are all spun up simultaneously:

```bash
ros2 launch xarm_planner_evaluation evaluation.launch.py
```

### Configuration Flags (Launch Arguments)

When running `evaluation.launch.py`, you can append several parameters via the CLI to customize the execution. 

| Flag / Parameter | Default Value | Description                                                                                                                                                                                                                                                                        |
| :--- | :--- |:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `planner_type` | `joint_space_apf_planner` | Name of the planner node executable to launch (e.g., `joint_space_apf_planner`, `virtual_obstacle_apf_planner`, `virtual_hills_apf_planner`, `vortex_apf_planner`, `guided_vortex_apf_planner`, `beetle_apf_planner`, `pso_apf_planner`, `hybrid_apf_planner`). |
| `scenarios_to_run` | `5` | Number of scenarios to execute during the current evaluation run.                                                                                                                                                                                                                  |
| `random_selection` | `False` | Whether to pick scenarios randomly instead of sequentially.                                                                                                                                                                                                                        |
| `total_scenarios` | `100` | Total number of scenarios generated in the JSON.                                                                                                                                                                                                                                   |

**Example Usage:**
To test a custom planner, run 10 scenarios, and pick them randomly from a pool of 100:
```bash
ros2 launch xarm_planner_evaluation evaluation.launch.py planner_type:=joint_space_apf_planner scenarios_to_run:=10 random_selection:=True total_scenarios:=100
```

---

## Results and Output

Upon completion of the evaluation scenarios, the `evaluator_node` compiles the metrics into a JSON file. 

These results are stored in the `results/` directory at the root of the workspace (e.g., `results/evaluation_result-0.1-beetle.json`).

These JSON files contain detailed per-scenario statistics, including:
* Success / Failure status
* Planning time
* Execution time
* Path length (joint space and task space)
* Collision events