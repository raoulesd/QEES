#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <memory>
#include <iostream>		
#include <unistd.h>		
#include <time.h>		
#include <sys/mman.h>	
#include <sched.h>		
#include <string>
#include <list>

#define LIST_NUM 10
#define EVAL_NUM 500
#define QoS_Policy 1 // 1 means "reliable", 2 means "best effort", 3 means "history"
#define RUN_REAL_TIME

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

static const rmw_qos_profile_t rmw_qos_profile_reliable = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  100,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
};

static const rmw_qos_profile_t rmw_qos_profile_best_effort = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  1,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE
};

static const rmw_qos_profile_t rmw_qos_profile_history = {
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  100,							// depth option for HISTORY
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
};

//int i, count = -1, init_num_int;
//double  subscribe_time[EVAL_NUM];

//struct timespec tp1;		// for clock
//FILE *fp;					// for file io

struct ListenerState {
    std::string name;
    std::string output_filename = "./evaluation/subscribe_time/subscribe_time_256";
    int count = -1;
    int init_num_int = 0;
    int i;
    int eval_loop_count = 0;
    double subscribe_time[EVAL_NUM];
    struct timespec tp1;
    FILE *fp = nullptr;
};

// eval_loop_count is updated when evaluation ends in each data size.
// 0 means evaluation of 256 bytes, 1 means evaluation of 512 bytes, ... 
//int eval_loop_count = 0;

void chatterCallback(const std_msgs::msg::String::SharedPtr msg, std::shared_ptr<ListenerState> listener_state){

//  RCLCPP_INFO(rclcpp::get_logger("listener"), "I heard: '%s'", msg->data.c_str());
  if (clock_gettime(CLOCK_REALTIME,&listener_state->tp1) < 0) {
	 perror("clock_gettime begin");
  }
  double receive_time = (double)listener_state->tp1.tv_sec + (double)listener_state->tp1.tv_nsec/ (double)1000000000L;

  std::string termination = msg->data.c_str();
  if (termination.find("end") != std::string::npos) {
  	printf("---end evaluation---\n");
	rclcpp::shutdown();
  }
  
  if (listener_state->count == -1) {

	// Initialize count
	char init_num_char = *( msg->data.c_str());
	char *init_num_pt = &init_num_char;
    listener_state->count = atoi(init_num_pt);
    listener_state->init_num_int = listener_state->count;	// if init_num_int is not 0, some messages are lost.

	// printf("first recieved number: %d \n\n", listener_state->count);
	printf("message loss of %s : %d \n", listener_state->name.c_str(), listener_state->init_num_int);
	printf("eval_loop %d of %s\n", listener_state->eval_loop_count, listener_state->name.c_str());
  }
   
  // evaluation
  if (listener_state->count < EVAL_NUM - 1) {
      listener_state->subscribe_time[listener_state->count] = receive_time;

//	 printf("%18.9lf\n",subscribe_time[count]);
//	 printf("I heard: [%c]\n",* ( msg->data.c_str()) );
//	 printf("I heard: [%s]\n", msg->data.c_str());
//	 printf("Time Span:\t%ld.%09ld\n", listener_state->tp1.tv_sec, listener_state->tp1.tv_nsec);
//	 printf("subscribe_time[%2d]:\t%18.9lf\n", count,subscribe_time[count]);
//
//	 char* p = (char *) msg->data.c_str();
//	 p++;
//	 printf("I heard: [%c%c]\n",* ( msg->data.c_str()), *p);

      listener_state->count++;
  } 
  else if (listener_state->count == EVAL_NUM - 1) {
    listener_state->subscribe_time[listener_state->count] = receive_time;

    std::string suffix = "byte_" + listener_state->name + ".txt";

	// Output subscribe_time[] collectively to subscribe_tim_*bytee.txt after evaluation
	if ((listener_state->fp = fopen((listener_state->output_filename + suffix).c_str(), "w")) != NULL) {
	
	  // write init_num
	  if (fprintf(listener_state->fp, "%d\n",listener_state->init_num_int ) < 0) {
		// Write error
		printf("error : can't output %s\n", (listener_state->output_filename + suffix).c_str());

	  }

	  // write subscribe_time[]
	  for (listener_state->i = 0; listener_state->i < EVAL_NUM; listener_state->i++) {
		if (fprintf(listener_state->fp, "%18.9lf\n", listener_state->subscribe_time[listener_state->i]) < 0) {
		  // Write error
		  printf("error : can't output %s\n", (listener_state->output_filename + suffix).c_str());
		  break;
		}
	  }

	  // printf("output data %d \n", listener_state->eval_loop_count);

	  fclose(listener_state->fp);
	} 
	else {
	  printf("error : can't output subscribe_time_*byte.txt'");
	}
	
	// Initialization of evaluation
    listener_state->count = -1;					    // initialize for next date size
    listener_state->eval_loop_count++;				// update for next data size

	if( listener_state->eval_loop_count == 1){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_512";
	}else if( listener_state->eval_loop_count == 2){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_1K";
	}else if( listener_state->eval_loop_count == 3){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_2K";
	}else if( listener_state->eval_loop_count == 4){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_4K";
	}else if( listener_state->eval_loop_count == 5){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_8K";
	}else if( listener_state->eval_loop_count == 6){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_16K";
	}else if( listener_state->eval_loop_count == 7){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_32K";
	}else if( listener_state->eval_loop_count == 8){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_64K";
	}else if( listener_state->eval_loop_count == 9){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_128K";
	}else if( listener_state->eval_loop_count == 10){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_256K";
	}else if( listener_state->eval_loop_count == 11){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_512K";
	}else if( listener_state->eval_loop_count == 12){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_1M";
	}else if( listener_state->eval_loop_count == 13){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_2M";
	}else if( listener_state->eval_loop_count == 14){
      listener_state->output_filename = "./evaluation/subscribe_time/subscribe_time_4M";
	}else if( listener_state->eval_loop_count == 15){
	  // End of measurement
      listener_state->count = EVAL_NUM;
	}
	
  }
}

int main(int argc, char * argv[])
{
    mlockall(MCL_FUTURE);		// lock all cached memory into RAM and prevent future dynamic memory allocations

    usleep(1000);

#ifdef RUN_REAL_TIME
    sched_param  pri = {94};
    if (sched_setscheduler(0, SCHED_FIFO, &pri) == -1) { // set FIFO scheduler
        perror("sched_setattr");
        exit(EXIT_FAILURE);
    }
#endif

    rclcpp::init(argc, argv);

    std::list<std::shared_ptr<rclcpp::Node>> nodes;
    std::vector<std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>> subscriptions;

    rclcpp::executors::MultiThreadedExecutor executor;

    // Create a vector to store listener states for each listener
    std::vector<std::shared_ptr<ListenerState>> listener_states;
    for(int l = 0; l < LIST_NUM; ++l) {

        std::string node_name = "listener_" + std::to_string(l);
        auto node = rclcpp::Node::make_shared(node_name);
        auto listener_state = std::make_shared<ListenerState>();
        listener_state->name = node_name;

        // geting parameters from yaml file
        node->declare_parameter("QoS_Policy");
        int QoS_Policy_param;
        node->get_parameter_or("QoS_Policy", QoS_Policy_param, QoS_Policy);

        // QoS settings
        rmw_qos_profile_t custom_qos_profile;
        if (QoS_Policy_param == 1) {
            custom_qos_profile = rmw_qos_profile_reliable;
        } else if (QoS_Policy_param == 2) {
            custom_qos_profile = rmw_qos_profile_best_effort;
        } else if (QoS_Policy_param == 3) {
            custom_qos_profile = rmw_qos_profile_history;
        }

        auto sub = node->create_subscription<std_msgs::msg::String>(
                "chatter",
                [listener_state](const std_msgs::msg::String::SharedPtr msg) {chatterCallback(msg, listener_state);},
                custom_qos_profile
                );

        nodes.push_back(node);
        subscriptions.push_back(sub);

        executor.add_node(node);
    }

    printf("start evaluation\n");

#pragma GCC diagnostic pop

    executor.spin();


    return 0;
}
