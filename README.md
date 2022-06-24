# Button manager

## Overview
This node manages DI signals from`/dio_ros_driver` as button manipulation.

This has a noise reduction from GPIO signals.
It aim to prevent malfunction of button manipulation.

## Input and Output
- input
  - from [dio_ros_driver](https://github.com/tier4/dio_ros_driver)
    - `/dio/din[0-3]` : GPIO input topic. It is output at regular intervals regardless of whether the button is ON or OFF.
- output
  - to [engage_srv_converter](https://github.com/eve-autonomy/engage_srv_converter) and [autoware_state_machine](https://github.com/eve-autonomy/autoware_state_machine)
    - `(launch_arg button_name)_manager/output/(launch_arg button_name)` : Output indicating that the button is pressed.

## Node Graph
In this figure, `/engage_button_manager` and `/delivery_reservation_button_manager` are both execution node names of this node.
The name is changed with  a launch argument `button_name`.
![node graph](http://www.plantuml.com/plantuml/proxy?src=https://raw.githubusercontent.com/eve-autonomy/button_manager/docs/node_graph.pu)

## Launch arguments
<table>
  <thead>
    <tr>
      <th scope="col">Name</th>
      <th scope="col">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>button_name</td>
      <td>This argument modifies the node name, namespace, and topic name.</td>
    <tr>
    <tr>
      <td>port_name</td>
      <td>This argument specifies the GPIO port name.</td>
    <tr>
  </tbody>
</table>

## Parameter description

### Output signal parameters
<table>
  <thead>
    <tr>
      <th scope="col">Name</th>
      <th scope="col">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>active_polarity</td>
      <td>This inverts mutually high (ON) and low (OFF) values of a DI signal.</td>
    <tr>
  </tbody>
</table>

### GPIO noise reduction parameters
<table>
  <thead>
    <tr>
      <th scope="col">ID</th>
      <th scope="col">Name</th>
      <th scope="col">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>T0</td>
      <td>not_pressed_period_threshold</td>
      <td>This defines a minimum time, until it takes for a button to be recognized as NOT pressed.</td>
    <tr>
    <tr>
      <td>T1</td>
      <td>pressed_period_threshold</td>
      <td>This defines a minimum time, until it takes for a button to be recognized as PRESSED.</td>
    <tr>
    <tr>
      <td>T2</td>
      <td>not_pressed_period_threshold_after_pressed</td>
      <td>This defines a minimum time to move to next button action, until it takes for a button to be recognized as not pressed.</td>
    <tr>
    <tr>
      <td>T3</td>
      <td>max_allowable_period_of_pressing</td>
      <td>This defines the maximum expected press period.</td>
    <tr>
  </tbody>
</table>

## Button press state transition
![state_machine](http://www.plantuml.com/plantuml/proxy?src=https://raw.githubusercontent.com/eve-autonomy/button_manager/docs/state_machine.pu)

