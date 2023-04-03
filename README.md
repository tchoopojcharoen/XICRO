

# XICRO: Firmware & Middleware Code Genration for Interfacing any-bit Microcontroller with ROS2

(See the document here: https://xicro-ros2.readthedocs.io/en/latest/)

## Rationale behind "xicro":
The Robot Operating System (ROS) is a popular open-source framework for developing robotic systems. ROS2, the latest version of the framework, is designed to support real-time, distributed systems, and has become a popular choice for robotics researchers and developers. However, one limitation of ROS2 is that it is designed to run on high-performance computers, which can be expensive and impractical for many robotics applications.

To address this limitation, several middleware solutions have been developed to allow ROS2 to run on microcontrollers with limited resources. One such solution is MicroROS, a middleware that provides a communication layer between microcontrollers and ROS2. However, MicroROS is limited to microcontrollers with 32-bit architecture, and cannot be used with lower-bit architecture such as AVR. This limitation poses a challenge for developers who want to use ROS2 with low-cost, low-power microcontrollers that are widely available in the market.

To overcome this limitation, a software library is needed that allows any-bit architecture microcontroller to connect with ROS2 using UART or UDP. This library should provide a lightweight and efficient communication protocol that is optimized for microcontrollers with limited resources. By enabling low-cost microcontrollers to connect with ROS2, this library can help democratize access to ROS2 and make it more accessible to developers working on low-cost robotics projects.

## General features of "xicro":
* xicro auto-generates a firmware library for a microcontroller (.h file)
* xicro auto-generates a Python executable for communicating between the microcontroller and ROS2 network.
* xicro (should) supports all Arduino, ESP32, ESP8266 and STM32 families.

"xicro" is a part of an ongoing project of CoXsys Robotics under GPL license. 


## Installation
  - ### set up meta-pacakge for code generation
   To use XICRO, you must create a meta-package for the code generation called "Xicro" and this repository to that meta-package. Assume that your workspace is named [xxx_ws] and located in the Home directory.
  ```bash
    cd ~/xxx_ws/src      #cd to your workspace
    mkdir Xicro          #create metapackage
    cd Xicro             #cd to metapackage
    git clone https://github.com/imchin/Xicro .
    cd ~/xxx_ws
    colcon build
    source ~/xxx_ws/install/setup.bash
  ```
  - ### install python library
    The code generation feature of XICRO relies on certain Python libraries. If you are not using "serial" in any of your project, you may run the following lines of commands. Otherwise, the library name has to change.
    ```bash
    pip3 uninstall serial
    pip3 install pyserial
    pip3 install numpy
    ```
## Generated Contents
    XICRO allows user to generate 2 things.
    1.) ROS2-interface firmware library for the desired MCU
    2.) Python Executable for interfacing the desired MCU with ROS2 network 
## Setup Parameters in the Configuration File
    To configure proper code generation, you must modify a configuration file called "setup_xicro.yaml", which is located in the config folder of xicro_pkg package under the meta-package. Use your favorite editor to modify the file. (In this case, we use Visual Studio code).
    ```bash
    cd ~/xxx_ws/src/Xicro/xicro_pkg/config      
    code setup_xicro.yaml
    ```

## Configuration file: setup_xicro.yaml  

In this configuration file, you can modify the following parameters to match your needs. 
  - ### microcontroller
      
    - 1\. idmcu : This sets the ID of the MCU of the generated contents. [0-15] 
    
    - 2\. namespace : This is the name of the file that will be generated, i.e. "imu"

    - 3\. generate_library_Path : This is the location where the generated firmware library will be put in.
    
      XICRO will create a folder with files at this path.

        Path reference from ~/
        
    - 4\. connection
        
        - 1\. type : Support 2 mode 1.)Serial UART 2.)Wifi UDP (Only arduino and esp32)
          
        - 2\. serial_port : Name of the open serial port of the MCU
      
        - 3\. baudrate : This affects the data transmission rate. Users can use <=2000000
      
            \**Recommended at 115200 bit/s.

        - 4\. ip_address_mcu : IP address of mcu in WLAN
          
        - 5\. udp_port_mcu : port of connection UDP mode
        
  - ### ros : setup ros  
           
    - 1\. publisher : Configuration for publishing to a topic from MCU to ROS2.
        
        In format : [ [ID_topic,Name_topic,Interface],[ID_topic_2,Name_topic_2,Interface_2],.......]
        - 1\. ID_topic : This sets the ID of the published topic [0 to 255]
        - 2\. Name_topic : Specify the topic name you want MCU to publish to ROS2.
        - 3\. Interface : variable interface file configuration for use in the topic, which must be in the following format. 

      "(The name of the package that contains the interface file)/(Name_of_fileinterface).msg" 

    - 2\. subscriber : Configuration for subscribing to a topic from ROS2 to MCU.
        
        In format : [ [ID_topic,Name_topic,Interface],[ID_topic_2,Name_topic_2,Interface_2],.......]
  
        - 1\. ID_topic : It sets the ID of the subscribed topic [0 to 255].
        - 2\. Name_topic : Specify the topic name you want MCU subscribe from ROS2.
        - 3\. Interface : variable interfacefile configuration for use in the topic, which must be in the following format. 
        
        "(The name of the package that contains the interface file)/(Name_of_fileinterface).msg" 

    - 3\. srv_client : Configuration for service client. 
        
        In format : [ [ID_service,Name_service,Interface,time_out],[ID_service_2,Name_service_2,Interface_2,time_out_2],.......]

      - 1\. ID_service : It sets the ID of the service_client [0 to 255]
      - 2\. Name_service : Specify the servive name you want MCU service call to ROS2.
      - 3\. Interface : variable interfacefile configuration for use in the service, which must be in the following format. 
        
        "(The name of the package that contains the interface file)/(Name_of_fileinterface).srv"
        
      - 4\. time_out : Limit the maximum service usage time. (In type float)

    - 4\. srv_server : Configuration for service server. 
        In format : [ [ID_service,Name_service,Interface,time_out],[ID_service_2,Name_service_2,Interface_2,time_out_2],.......]

      - 1\. ID_service : It sets the ID of the service_server [0 to 255].
      - 2\. Name_service : Specify the servive name you want  ROS2 service call to MCU.
      - 3\. Interface : variable interfacefile configuration for use in the service, which must be in the following format. 
        
        "(The name of the package that contains the interface file)/(Name_of_fileinterface).srv"
      - 4\. time_out : Limit the maximum service usage time. (In type float)

    - 5\. action_client : Configuration for action client. 
        In format : [ [ID_action,Name_action,Interface,time_out],[ID_action_2,Name_action_2,Interface_2,time_out_2],.......]

      - 1\. ID_action : It sets the ID of the action_client [0,255]
      - 2\. Name_action : Specify the action name you want MCU action send_goal to ROS2.
      - 3\. Interface : variable interfacefile configuration for use in the action, which must be in the following format. 
        
        "(The name of the package that contains the interface file)/(Name_of_fileinterface).action"        
      - 4\. time_out : Limit the maximum action usage time. (In type float)        
      
    - 6\. action_server : Configuration for action server. 
        In format : [ [ID_action,Name_action,Interface,time_out],[ID_action_2,Name_action_2,Interface_2,time_out_2],.......]

      - 1\. ID_action : It sets the ID of the action_server [0 to 255]
      - 2\. Name_action : Specify the action name you want ROS2 action send_goal to MCU.
      - 3\. Interface : variable interfacefile configuration for use in the action, which must be in the following format. 
        
        "(The name of the package that contains the interface file)/(Name_of_fileinterface).action" 
      - 4\. time_out : Limit the maximum action usage time. (In type float)        
      
      

## Generating Firmware library
The library will be generated based on setup_xicro.yaml. The firmware library .h , .cpp will be created at $generate_library_Path according to setup_xicro.yaml.
```bash
  cd ~/xxx_ws/src      #cd to your workspace
  colcon build
  ros2 run xicro_pkg generate_library.py -mcu_type -module_name 
```
  -mcu_type (require) : Users must specify the family of MCU [arduino , esp , stm32]
      
  -module_name : HAL library
    
  - If you don't use stm32 family, ignore this argument. 
  
    ### Example 
    ARDUINO
    ```bash
        ros2 run xicro_pkg generate_library.py -mcu_type arduino  // Example generate for arduino family.
    ```
    ESP
    ```bash
        ros2 run xicro_pkg generate_library.py -mcu_type esp  // Example generate for esp family.
    ```
    STM32F411RE
    ```bash
        ros2 run xicro_pkg generate_library.py -mcu_type stm32 -module_name "stm32f4xx_hal.h"  // Example generate for stm32F4xx
    ```
## Generating Python Executable
The Python executabe will be generated based on "setup_xicro.yaml". The executable will be generated at path ~xxx_ws/src/Xicro/xicro_pkg/scripts.

```bash 
    ros2 run xicro_pkg generate_xicro_node.py -mcu_type arduino
```
  -mcu_type (require) : Users must specify the family of MCU [arduino , esp , stm32]

  The entry point is automatically added by the command.


## General Pipeline for using XICRO
- Generate the firmware library

- Generate the Python Executable
 
- Write and upload your firmware to the MCU

- Connect MCU to the computer

- Verify the permission for the open port 
  
- Execute the generated Python executable (run the generated node)
