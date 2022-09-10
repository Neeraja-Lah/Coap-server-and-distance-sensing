
**********************CSE522-Real Time Embedded Systems**********************
*****************************Assignment-3*************************************


Name - Neeraja Lahudva Kuppuswamy
ASU ID - 1224187432

Description : 

In this assignment, we have developed an application program on Zephyr that acts as a COAP Server and the system as client. We use Copper for Chrome COAP user-agent for the COAP Uri Path to interact with the IOT devices and test the coap server. The following resources are created on the server:

1. Two HC-SR04 sensors for reporting the distance measurements of the target object 
2. An RGB Led


******************************************************************************
*****************Steps to compile and execute the code*****************


1. Unzip the RTES-LahudvaKuppuswamy-N_03.zip in the zephyrproject directory

2. To build, run west build -b mimxrt1050_evk project_3

3. Run west -v flash
	
4. Open putty and select port /dev/ttyACM0 and enter baud rate 115200

5.To generate the patch file, use the following command:
	diff –rauN --exclude=samples /(original zephyr tree) /(modified zephyr tree) > patch_project3


******************************************************************************
*****************Steps to discover resources on the server and perform actions on them*****************

1. Enter the location of the COAP Server by providing its URI

2. /.well-known/core: click on Discover button to retrieve the resources on the server from  /.well-known/core 

3. At the particular resource location, we can click on GET, POST, PUT, DELETE buttons to perform corresponding actions on them

4. /sensor/hcsr_i: hit the GET button to get the current distance measurement from either of the sensors (0 or 1) and Observe button to observe if there is a change in distance larger than 0.5 inches

5. /sensor/period: click PUT button to set the sampling period for the sensors in milliseconds

6. /led/led_n: click PUT button to turn on or off the LED (n = r, g & b) and GET button retrieves the current LED status


	

 

