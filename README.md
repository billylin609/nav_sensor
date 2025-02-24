# Install

```
sudo docker build -t ros-nav .
```

# Setup

**Docker**

```bash
sudo docker run --net host --privileged -it ros-nav

# All the other terminal
sudo docker ps
sudo docker exec -it <container-id> /bin/bash 
```

**terminal 1**

```bash
source /entrypoint.bash

roscore
# Note: press ctrl-c imediately after launch roscore to interupt, otherwise roscore will not be launched
```

**terminal 2**

```bash
export LCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1

source /entrypoint.bash

rosrun vectornav_listener ins_listener.py
```

**terminal 3**

```bash
source /entrypoint.bash

roslaunch src/vectornav/launch/vectornav.launch
```

Testing command


```
rostopic pub -r 1 /vectornav/ins vectornav/Ins "{
  header: { stamp: now, frame_id: 'base_link' },
  time: 1234.56,
  week: 123,
  utcTime: 987654321,
  insStatus: 2,
  yaw: 1.23,
  pitch: 2.34,
  roll: 3.45,
  latitude: 37.4275,
  longitude: -122.1697,
  altitude: 30.0,
  nedVelX: 0.1,
  nedVelY: 0.2,
  nedVelZ: 0.3,
  attUncertainty: [0.01, 0.01, 0.01],
  posUncertainty: 0.05,
  velUncertainty: 0.02
}"

```
