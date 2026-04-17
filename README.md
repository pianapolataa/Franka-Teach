# Franka-Teach

Bi-Manual Franka 3 robot setup.


## NUC Setup

1. Install Ubuntu 22.04 and a real-time kernel
2. Make sure the NUC is booted with the real-time kernel [[link](https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)].


## Workstation Machine Setup (skip if using bimanual nuc, it's already installed)

todo: how to setup network, etc.


1. Setup deoxys_control. NOTE: When doing `./InstallPackage`, select `0.13.3` for installing libfranka:

```bash
git clone git@github.com:NYU-robot-learning/deoxys_control.git
mamba create -n "franka_teach" python=3.10
conda activate franka_teach
cd deoxys_control/deoxys

# Instructions from deoxys repo (this takes a while to build everything)
./InstallPackage
make -j build_deoxys=1
pip install -U -r requirements.txt
```

2. Install the Franka-Teach requirements:

```bash
cd /path/to/Franka-Teach
pip install -r requirements.txt
```

3. Install ReSkin sensor library:

```bash
git clone git@github.com:NYU-robot-learning/reskin_sensor.git
cd reskin_sensor
pip install -e .
```


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

2. Run ssh tunneling: 
```
ssh -NL localhost:8001:192.168.100.203:443 bimanual-nuc 
ssh -NL localhost:8000:192.168.100.202:443 bimanual-nuc
```
Go to localhost:8000/desk and localhost:8001/desk for the Franka Desk.

The following credentials are used for the Franka Desk interface:

```
Username: GRAIL
Password: grail1234
```

NOTE: Franka Desk is cursed. You might face all sorts of issues with it. General troubleshooting:

- If it doesn't seem to connect, keep refreshing. If you lose all hope, reboot the robot.
- If end-effector doesn't show as active, you can go to the settings page and do a power off/on for the end-effector. You need to re-initialize the gripper in the same page after doing a power cycle.
- Two desk pages (for two robots) cannot be open at the same time. Close one tab and connect to the other one.

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

6. From the workstation, start servers:

```bash
cd /path/to/Franka-Teach/
python franka_server.py --config-name franka_server_right 
python franka_server.py --config-name franka_server_left
python port_split.py # Splits Oculus input to the left and right arm ports
```


## How to teleoperate

1. Do the steps until 6 in the "How to run the Franka-Teach environment" section.


2. Also, start the teleoperation script. Set the teleop mode based on if you are collecting human or robot demonstrations.:

```bash
python teleop.py 
```
Note: this bimanual teleop script does not have a stop control button, so make sure you have someone next to you helping stop the script.
