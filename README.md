# multi_uav_se_mission

This package presents the software solution for simulating search and task execution applications using multiple UAVs. In the proposed scenario, marks are placed on the ground where the UAVs searchers perform a search mission to identify them and then assign the tasks to the UAVs workers, responsible for performing these identified tasks.

## Installing dependencies

```sh
sh ~/drone_simulator_ws/src/multi_uav_se_mission/scripts/install/install_dependencies.sh
```

## Running the simulation

```sh
source ~/drone_simulator_ws/src/multi_uav_se_mission/scripts/start/simulation_test.sh <experimentId>
```
\<experimentId\>: is the experiment number, used to identify bag files.
