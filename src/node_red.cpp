#include "node_red.h"
#include "complementary-filter.h"
#include "confidential.h"
#include "controller.h"
#include <PubSubClient.h>
#include <WiFi.h>
#include <stdio.h>
#include <stdlib.h>

void reconnect();
void node_red_task(void *);
void node_red_starter_task(void *);
bool node_red_sub_to_topics();
bool node_red_mqtt_connect();

// Maybe implement stop if X/Y go too weird?

WiFiClient espClient;
PubSubClient mqtt_client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
bool started = false;
bool connected = false;
bool publish_task_publish = true;

void callback(char *topic, byte *message, unsigned int length) {
  // This is needed to convert the byte message to a char message
  char messageTemp[length];
  for (int i = 0; i < length; i++) {
    messageTemp[i] = (char)message[i];
  }

  // Enables or disables publising controller and complementary filter
  if (strcmp(topic, NODE_RED_ENABLE_TOPIC) == 0) {
    publish_task_publish = !publish_task_publish;
    printf("node-red publish is set to: %d\n", publish_task_publish);
  }
  // The following 3 if statments sets orientation pid parameters
  else if (strcmp(topic, NODE_RED_SET_ORIENTATION_P_TOPIC) == 0) {
    float p = atof(messageTemp);
    if (controller_set_orientation_p(p)) {
      printf("Set orientation pid p to: %f.\n", p);
    } else
      printf("Failed to set orientation pid p.\n");
  } else if (strcmp(topic, NODE_RED_SET_ORIENTATION_I_TOPIC) == 0) {
    float i = atof(messageTemp);
    if (controller_set_orientation_i(i))
      printf("Set orientation pid i to: %f.\n", i);
    else
      printf("Failed to set orientation pid i.\n");
  } else if (strcmp(topic, NODE_RED_SET_ORIENTATION_D_TOPIC) == 0) {
    float d = atof(messageTemp);
    if (controller_set_orientation_d(d))
      printf("Set orientation pid d to: %f.\n", d);
    else
      printf("Failed to set orientation pid d.\n");
  } else if (strcmp(topic, NODE_RED_GET_ORIENTATION_PID_TOPIC_SEND) == 0) {
    char pid_msg[100]; // 100 is probably big enough
    sprintf(pid_msg, "p:%f i:%f d:%f", controller_get_NW().Kp,
            controller_get_NW().Ki, controller_get_NW().Kd);
    node_red_publish(NODE_RED_GET_ORIENTATION_PID_TOPIC_RECEIVE, pid_msg);
  }
}

void node_red_publish(const char *topic, const char *data) {
  if (!mqtt_client.connected()) {
    mqtt_client.connect("Quadcopter");
    node_red_sub_to_topics();
  }
  if (!(started && connected)) {
    // printf("Node red can't publish because it's not connected to wifi.\n");
    return;
  }
  if (!mqtt_client.publish(topic, data)) {
    // printf("failed to send %s to %s\n", data, topic);
    connected = false;
  } else {
    // printf("successfully sent %s to %s\n", data, topic);
  }
}

void node_red_publish_controller_info() {
  char number_c[50];
  sprintf(number_c, "%f", controller_get_NE().throttle);
  node_red_publish("NE", number_c);
  sprintf(number_c, "%f", controller_get_SE().throttle);
  node_red_publish("SE", number_c);
  sprintf(number_c, "%f", controller_get_NW().throttle);
  node_red_publish("NW", number_c);
  sprintf(number_c, "%f", controller_get_SW().throttle);
  node_red_publish("SW", number_c);

  sprintf(number_c, "%f", get_X());
  node_red_publish("X", number_c);
  sprintf(number_c, "%f", get_Y());
  node_red_publish("Y", number_c);
}

void node_red_start() {
  xTaskCreatePinnedToCore(node_red_starter_task, "node-red-start",
                          configMINIMAL_STACK_SIZE * 5, NULL, 1, NULL, 1);
}

void node_red_starter_task(void *) {
  started = true;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  printf("gonna connect to wifi %s with password %s\n", WIFI_SSID,
         WIFI_PASSWORD);
  uint8_t connect_attemt_counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    connect_attemt_counter += 1;
    delay(500);
    Serial.print(".");
    if (connect_attemt_counter > NODE_RED_CONNECT_ATTEMPT_MAX) {
      connected = false;
      printf("Failed to connect to wifi.\n");
      vTaskDelete(NULL);
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mqtt_client.setServer(MQTT_BROKER, 1883);
  mqtt_client.setCallback(callback);

  if (!node_red_mqtt_connect()) {
    connected = false;
    printf("Failed to connect to broker.\n");
    vTaskDelete(NULL);
  }

  connected = true;
  printf("\nConnected to broker\n");
  printf("Node red started\n");

  node_red_publish("NW", "HELLO WORLD");
  xTaskCreatePinnedToCore(node_red_task, "node-red-controller-publish-task",
                          configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL, 1);

  vTaskDelete(NULL);
}

// Handles publishing controller and filter data via node-red and makes sure
// received mqtt messages are handled.
void node_red_task(void *) {
  for (;;) {
    if (publish_task_publish && !controller_stopped()) {
      node_red_publish_controller_info();
    }
    mqtt_client.loop();
    vTaskDelay(1.f / NODE_RED_PUBLISH_HZ * 1000);
  }
}

void node_red_stop_publishing_controller_info() {
  publish_task_publish = false;
}

void node_red_start_publishing_controller_info() {
  publish_task_publish = true;
}

// Attempt to connect to the broker
bool node_red_mqtt_connect() {
  uint8_t connect_attemt_counter = 0;
  while (!mqtt_client.connected()) {
    connect_attemt_counter += 1;
    mqtt_client.connect("Quadcopter");
    delay(500);
    Serial.print(".");
    if (connect_attemt_counter > NODE_RED_CONNECT_ATTEMPT_MAX) {
      return false;
    }
  }
  return node_red_sub_to_topics();
}

// Attempt to subscribe to the topics
bool node_red_sub_to_topics() {
  bool err = mqtt_client.subscribe(NODE_RED_ENABLE_TOPIC,
                                   1); // Can only subscribe at 0 or 1
  err &= mqtt_client.subscribe(NODE_RED_SET_ORIENTATION_P_TOPIC, 1);
  err &= mqtt_client.subscribe(NODE_RED_SET_ORIENTATION_I_TOPIC, 1);
  err &= mqtt_client.subscribe(NODE_RED_SET_ORIENTATION_D_TOPIC, 1);
  err &= mqtt_client.subscribe(NODE_RED_GET_ORIENTATION_PID_TOPIC_SEND, 1);
  return err;
}

// This code is not good as it will block the core from running other code
void reconnect() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect("arduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqtt_client.publish("test", "Reconnected!");
      // ... and resubscribe
      mqtt_client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
