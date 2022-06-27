# Button manager

## Overview
This node manages DI signals from`/dio_ros_driver` as button manipulation.

This has a noise reduction from GPIO signals.
It aim to prevent malfunction of button manipulation.

## Input and Output
- input
  - from [dio_ros_driver](https://github.com/tier4/dio_ros_driver)
    - `/dio/din[0-7]` \[[dio_ros_driver/msg/DIOPort](https://github.com/tier4/dio_ros_driver/blob/develop/ros2/msg/DIOPort.msg)\]:<br>GPIO input topic. It is output at regular intervals regardless of whether the button is ON or OFF.<br>Which of `din[0-7]` is chosen depends on the `port_name` of the [Launch arguments](#launch-arguments).
- output
  - to [engage_srv_converter](https://github.com/eve-autonomy/engage_srv_converter) and [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine)
    - `(button_name)_manager/output/(button_name)` \[[autoware_state_machine_msgs/msg/VehicleButton](https://github.com/eve-autonomy/autoware_state_machine_msgs/blob/main/msg/VehicleButton.msg)\]:<br>Output indicating that the button is pressed.<br>See [Launch arguments](#launch-arguments) for `button_name`.

## Node Graph
In this figure, `/engage_button_manager` and `/delivery_reservation_button_manager` are both execution node names of this node.
The name is changed with  a launch argument `button_name`.
![node graph](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/button_manager/main/docs/node_graph.pu)

## Launch arguments
|Name|Description|
|:---|:----------|
|button_name|This argument modifies the node name, namespace, and topic name.<br>Set the name `***_button` as the value.|
|port_name|This argument specifies the GPIO port name. Select `dio[0-7]` as the value.|

## Parameter description

### Output signal parameters

|Name|Description|
|:---|:----------|
|active_polarity|This inverts mutually high (ON) and low (OFF) values of a DI signal.<br>- True: With inversion. - False: No inversion.|

### GPIO noise reduction parameters
|ID|Name|Description|
|:-|:---|:----------|
|T0|not_pressed_period_threshold|This defines a minimum time, until it takes for a button to be recognized as NOT pressed.|
|T1|pressed_period_threshold|This defines a minimum time, until it takes for a button to be recognized as PRESSED.|
|T2|not_pressed_period_threshold_after_pressed|This defines a minimum time to move to next button action, until it takes for a button to be recognized as not pressed.|
|T3|max_allowable_period_of_pressing|This defines a maximum time, until it takes for a button to be recognized as one time pressure. If someone presses the button over the time, this node recognized as malfunction.|

## Button press state transition
![state_machine](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/eve-autonomy/button_manager/main/docs/state_machine.pu)

