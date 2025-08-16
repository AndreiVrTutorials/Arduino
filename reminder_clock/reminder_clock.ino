#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

// Create an instance of the ROS node handle
ros::NodeHandle nh;

// Pin for the red LED
const int RED_LED_PIN = 13;  // Red LED pin, set to pin 13 on the Arduino

// Callback function declaration (it must be declared before it is used)
void messageCallback(const std_msgs::Int32MultiArray& msg) {
  // Extract values from the received message
  int greenLedTime = msg.data[0];  // The first value controls the duration the red LED stays on
  int waitTime = msg.data[1];      // The second value controls how long to wait before the next action

  // Turn the red LED on
  digitalWrite(RED_LED_PIN, HIGH);  

  // Wait for the amount of time specified by greenLedTime (multiplied by 1000 to convert to milliseconds)
  delay(greenLedTime * 1000);  // Multiply by 1000 to convert from seconds to milliseconds
  
  // Turn the red LED off
  digitalWrite(RED_LED_PIN, LOW);

  // Wait for the amount of time specified by waitTime (multiplied by 1000 to convert to milliseconds)
  delay(waitTime * 1000);
}

// Subscriber (after declaring the callback function)
ros::Subscriber<std_msgs::Int32MultiArray> sub("led_control", &messageCallback);  // Subscribe to the "led_control" topic

// Publisher
std_msgs::Int32MultiArray msg;  // Create an Int32MultiArray message
ros::Publisher pub("led_control", &msg);  // Publish to the "led_control" topic

// Initialize an array to store the data for the message
int32_t data[2] = {2, 1};  // {red LED on time (2 seconds), wait time (1 second)}

void setup() {
  pinMode(RED_LED_PIN, OUTPUT);  // Set the red LED pin as an output

  // Initialize ROS node
  nh.initNode();

  // Advertise the publisher and subscribe to the subscriber
  nh.advertise(pub);  // Advertise the publisher to the ROS network
  nh.subscribe(sub);  // Subscribe to the topic to listen for messages

  // Initialize the message with the initial data values
  msg.data = data;
  msg.data_length = 2;  // Set the length of the message data (2 values)
}

void loop() {
  // Update message with current data values
  msg.data[0] = data[0];  // Set the first element (time the LED stays on)
  msg.data[1] = data[1];  // Set the second element (wait time before next action)

  // Publish the message to the ROS network
  pub.publish(&msg);

  // Quick blink of the red LED to indicate activity
  digitalWrite(RED_LED_PIN, HIGH);  // Turn the red LED on
  delay(300);  // Wait for 300 milliseconds (LED stays on)
  digitalWrite(RED_LED_PIN, LOW);  // Turn the red LED off
  delay(300);  // Wait for 300 milliseconds (LED stays off)

  // Wait for 5 seconds before repeating
  delay(5000);  // Wait for 5 seconds before sending the next message

  // Handle any incoming messages or requests
  nh.spinOnce();  // Process any incoming messages or ROS commands
}