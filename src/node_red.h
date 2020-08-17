#ifndef NODE_RED_H_
#define NODE_RED_H_

#define NODE_RED_CONNECT_ATTEMPT_MAX 5
#define NODE_RED_PUBLISH_HZ 20
#define NODE_RED_ENABLE_TOPIC "mqtt-enable-output"

void node_red_start();
void node_red_publish(const char* topic, const char* data);
void node_red_publish_controller_info();

#endif /*NODE_RED_H_*/