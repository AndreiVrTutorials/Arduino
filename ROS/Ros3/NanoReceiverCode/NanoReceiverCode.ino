#include <ros.h>
#include <std_msgs/String.h>
#include <SoftwareSerial.h>

// define pin assignments for serial communication
const uint8_t RX_PIN = 3;  // receive pin for SoftwareSerial 
const uint8_t TX_PIN = 4;  // transmit pin for SoftwareSerial
const uint32_t BAUD_RATE = 57600; // communication speed

// initialize SoftwareSerial with defined pins
SoftwareSerial softSerial(RX_PIN, TX_PIN);

// create a ROS node handle
ros::NodeHandle nh;

// callback function to handle incoming messages
void messageCb(const std_msgs::String& msg) {
  softSerial.println(msg.data);  // send received message to SoftwareSerial
}

// subscribe to the "chatter" topic and link it to the callback function
ros::Subscriber<std_msgs::String> sub("chatter", &messageCb);

void setup() {
  softSerial.begin(BAUD_RATE);  // start serial communication
  nh.initNode();  // initialize ROS node
  nh.subscribe(sub);  // subscribe to the topic
}

void loop() {
  nh.spinOnce();  // process incoming ROS messages
}
