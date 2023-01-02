
# Automatic turret (prototype)

  

Creators :

- Adrien STPIERRE

- Valentin ROSSETTI

  

Welcome on the repo about the prototype of our nerf automatic turret. This project was created during the "Prototyping" subject at CPE Lyon, over a period of 20 hours.

  

## Technologies used

The project uses :

- a webcam

- a ToF sensor

- an ESP32 T-Display

- a servomotor dynamixel AX-12

- a Nerf gun

  

We used the following technologies :

- ROS middleware

- Arduino language

- Bluetooth Low Energy

- 3D printing

- MIT app inventor (to create a mobile application)

- Darknet YOLO

  

## Startup

First, build the project in the folder "PROTO_ws" with the command `catkin build`

  

Upload the "vl53l0x.ino" in the ESP32.

  
  

Then you can launch the scripts.

Start with :

```

# launch and init the servomotors

roslaunch my_dynamixel_tutorial controller_manager.launch

roslaunch my_dynamixel_tutorial start_tilt_controller.launch

  

# launch the webcam

roslaunch usb_cam usb_cam-test.launch

  

# launch the person detection

roslaunch darknet_ros darknet_ros.launch

  

# if a person is detected, publish on a specific topic a boolean

rosrun process filter_person.py

  
  

# Launch the node for the rosserial connection between the ESP32 and the monitor

rosrun rosserial_python serial_node.py

  

# Launch the node of the turrell rotation

rosrun my_dynamixel_tutorial turn_servo.py

  

#Launch the node allowing the trigger to be pulled

rosrun my_dynamixel_tutorial tir_servo.py 

```

## Links to the youtube video
[Valentin's channel](https://youtu.be/gz8jxxPj0K0)
[Adrien's channel](https://youtu.be/nG4EB50BXwI)
