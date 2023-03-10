# WHEELTEC R550 Controller

轮趣科技R550无人车控制代码。

Analysis the function of multi agents reinforcement learning.

## Stage v1.x
We shall use UGV(Unmanned Ground Vehicle) for the first stage, 
UAV(Unmanned Aerial Vehicles) will be considered if outcomes are good.

In this stage, we shall only apply 1v1 environment.

### Stage v1.0

1 vs 1 experiment, one UGV is controlled by computer automatically, while another one is controlled by human operator.

* computing UWB data from 8 UWB nodes and 2 on each UGV, acquiring their position and velocity.
* use our reinforcement learning model to get strategy for the auto-UGV.
* send data to auto-UGV by Wi-Fi(for now, maybe a low-latency technology in the future).
* ROS communication or directly to STM32, change the parameters of rotors.
* repeat step1, go on.

If all things go well, this would function normally.

Warnings:
* data transferring latency(delay) in the whole process.

### Stage v1.1

