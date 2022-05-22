# airsim_controller
## Installation
```bash
git clone https://github.com/ozakiryota/airsim_controller.git
cd airsim_controller/docker
./build.sh
```

## Usage
### Randomize pose
```bash
$ cd airsim_controller/docker
$ ./interact.sh
$ ./drone_random_pose
```
### Flight according to waypoints
```bash
$ cd airsim_controller/docker
$ ./interact.sh
$ ./drone_waypoint_flight_withnoise
```