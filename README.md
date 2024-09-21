# Franka-Teach

Bi-Manual Franka 3 robot setup.


## NUC Setup

todo

1. Install Ubuntu 22.04 and a real-time kernel
2. Make sure the NUC is booted with the real-time kernel


## Franka Setup

todo


## Proxy Setup

1. Install FoxyProxy extension on Chrome or Firefox. Set up the proxy like this:

![Foxy Proxy](./imgs/foxy_proxy.png)

2. Setup NUC as an ssh host like this:

```bash
Host nuc
    HostName 10.19.248.70
    User robot-lab
    LogLevel ERROR
    DynamicForward 1337
```


## How to run the Franka-Teach environment

1. Ssh into the nuc:

```bash
ssh nuc
```

2. Go to `172.16.0.4/desk` for the Franka Desk interface for the right robot and `192.16.1.4/desk` for the left robot.

The following credentials are used for the Franka Desk interface:

```
Username: GRAIL
Password: grail1234
```
NOTE: Franka Desk interface may have a hard time connecting to the robot. If this happens, try refreshing the page until it does first. Also check if a Desk interface is open on another tab.

3. Open the brakes for the robot:

![open_brakes](./imgs/unlock_joints.png)

4. Enable FCI mode:

![fci](./imgs/fci.png)

5. Start the deoxys control process on the NUC:

```bash
cd /home/robot-lab/work/deoxys_control/deoxys
./auto_scripts/auto_arm.sh config/franka_left.yml # franka_right.yml for the right robot
./auto_scripts/auto_gripper.sh config/franka_left.yml # in a different terminal, if you want to use the gripper
```

6. From the Lambda, start servers:

```bash
cd /path/to/Franka-Teach/
cd frankateach
python3 franka_server.py
python3 camera_server.py # in a different terminal
```

7. Run franka_env test script:

```bash
cd /path/to/Franka-Teach/
python3 test_franka_env.py
```

## How to teleoperate

1. Do the steps until 6 in the "How to run the Franka-Teach environment" section.


2. Also, start the teleoperation scripts:

```bash
cd /path/to/Franka-Teach/
cd frankateach
# each in a different terminal
python3 oculus_stick.py
python3 teleoperator.py
```

3. If `save_states=True` in `FrankaOperator` class in `teleoperator.py`, the operation will not start until you start the data collection.
You can start the data collection by running the `collect_data.py` script:

```bash
cd /path/to/Franka-Teach/
python3 collect_data.py demo_num=0
```

4. Use the VR controllers to control the robot.
