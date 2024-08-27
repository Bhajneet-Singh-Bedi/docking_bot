/*
 *  Code initializes and connects to a WiFi network using given SSID and password,
 *  then publishes a "Hello World!" message to a ROS topic "chatter" at regular intervals.
 *  Make sure to update the SSID, password, IP and server details as per your network.
 *
 *  Create by Stan Fu on 2023/08/07
 */
#include <Arduino.h>
#include "WiFi.h"
#include <ros.h>
// #include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

IPAddress server(192, 168, 205, 237);
uint16_t serverPort = 11411;
const char *ssid = "realme";
const char *password = "voea4198";

// Be polite and say hello
char hello[13] = "hello world!";
uint16_t period = 1000;
uint32_t last_time = 0;


int coord1[2] = {4, 3};
int coord2[2] = {1, 2};
int coord3[2] = {5, 6};

ros::NodeHandle nh;
// Make a chatter publisher
// std_msgs::String str_msg;
geometry_msgs::Pose pose_msg;
ros::Publisher coord("docking_coord", &pose_msg);

void setupWiFi();

void setup()
{
  Serial.begin(115200);
  setupWiFi();

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("ROS IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  // Start to be polite
  nh.advertise(coord);
}

void loop()
{
  if (millis() - last_time >= period)
  {
    last_time = millis();
    if (nh.connected())
    {
      // Serial.println("Connected");
      // Say hello
      // str_msg.data = hello;
      
      pose_msg.position.x = 4.5;
      pose_msg.position.y = 2.5;
      coord.publish(&pose_msg);
    }
    else
    {
      Serial.println("Not Connected");
    }
  }
  nh.spinOnce();
  delay(1);
}

void setupWiFi()
{
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP:   ");
  Serial.println(WiFi.localIP());
}
