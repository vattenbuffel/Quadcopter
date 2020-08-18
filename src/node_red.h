#ifndef NODE_RED_H_
#define NODE_RED_H_

#define NODE_RED_CONNECT_ATTEMPT_MAX 5
#define NODE_RED_PUBLISH_HZ 50
#define NODE_RED_ENABLE_TOPIC "enable-output"
#define NODE_RED_SET_ORIENTATION_P_TOPIC "orientation-p"
#define NODE_RED_SET_ORIENTATION_I_TOPIC "orientation-i"
#define NODE_RED_SET_ORIENTATION_D_TOPIC "orientation-d"

void node_red_start();
void node_red_publish(const char *topic, const char *data);
void node_red_publish_controller_info();
void node_red_stop_publishing_controller_info();
void node_red_start_publishing_controller_info();

#endif /*NODE_RED_H_*/