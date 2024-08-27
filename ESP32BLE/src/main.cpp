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

// demo coordinates, which we'll get from BLE beacons.
float coord1[2] = {4.0, 3.0};
float coord2[2] = {1.0, 2.0};
float coord3[2] = {5.0, 6.0};
// demo rssi values of beacons.
float rssi_1 = 50, rssi_2 = 60, rssi_3 = 55;

struct dock_coord
{
  float x;
  float y;
};

ros::NodeHandle nh;
// Make a chatter publisher
// std_msgs::String str_msg;
geometry_msgs::Pose pose_msg;
ros::Publisher coord("docking_coord", &pose_msg);

void setupWiFi();
dock_coord dock_trilateration();
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
      dock_coord position = dock_trilateration();
      pose_msg.position.x = position.x;
      pose_msg.position.y = position.y;
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

dock_coord dock_trilateration()
{
  // (x - x1)^2 + (y - y1)^2 = d1^2
  // (x - x2)^2 + (y - y2)^2 = d2^2
  // (x - x3)^2 + (y - y3)^2 = d3^2
  float A = 2 * (coord2[0] - coord1[0]);
  float B = 2 * (coord2[1] - coord1[1]);
  float C = rssi_1 * rssi_1 - rssi_2 * rssi_2 - coord1[0] * coord1[0] - coord1[1] * coord1[1] + coord2[0] * coord2[0] + coord2[1] * coord2[1];

  float D = 2 * (coord3[0] - coord2[0]);
  float E = 2 * (coord3[1] - coord2[1]);
  float F = rssi_2 * rssi_2 - rssi_3 * rssi_3 - coord2[0] * coord2[0] - coord2[1] * coord2[1] + coord3[0] * coord3[0] + coord3[1] * coord3[1];

  float x = (C * E - F * B) / (E * A - B * D);
  float y = (C * D - A * F) / (B * D - A * E);

  return {x, y};
}
