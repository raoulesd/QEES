#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "std_msgs/msg/string.hpp"

#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <stdio.h>

#include <time.h>
#include <unistd.h>

#include <sys/mman.h>		
#include <sched.h>


#define EVAL_NUM 120	// evaluation number for each data size
#define PUBLISH_Hz 10
#define QoS_Policy 3	// 1 means "reliable", 2 means "best effort", 3 means "history"
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

struct timespec tp1; //timespec for publish_time
struct timespec tp2; // timespec for ack_time
int i, count = -1;	// count is current evaluation number (< EVAL_NUM)
double publish_time[EVAL_NUM];
double ack_time[EVAL_NUM];

std::string s, bytedata;
FILE *fp;				

// A function that takes a file name as an argument and returns the contents of the file
std::string read_datafile(std::string message_filename) {

  // Read data from data_ * byte.txt to std :: string bytedata
  std::ifstream ifs(message_filename.c_str());
  if (ifs.fail()) {
	 std::cerr << "data_*byte.txt do not exist.\n";
	 exit(0);
  }
 
  std::string bytedata;
  getline(ifs, bytedata);

  return bytedata;
}

// EVAL_NUM times, publish message_filename (data_ * byte.txt) and output time to output_filename (publish_time_ * byte.txt)
int eval_ros2(std::string message_filename, std::string output_filename, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_pub){
if (-1 < count) {

	auto msg = std::make_shared<std_msgs::msg::String>();
	std::stringstream ss;
  
	// Create message
	ss << count;		
	s = ss.str() + bytedata;
	msg->data = s;
  
	// Time recording publish_time
	if (clock_gettime(CLOCK_REALTIME,&tp1) < 0) {
	  perror("clock_gettime begin");
	  return 0;
  }
  publish_time[count] = (double)tp1.tv_sec + (double)tp1.tv_nsec/ (double)1000000000L;

	// printf("%18.9lf\n",publish_time[count]);
	// printf("publish_time[%2d]:\t%18.9lf\n", count, publish_time[count]);
	// printf("I say: [%s]\n", msg->data.c_str());
	// printf("Time Span:\t%ld.%09ld\n", tp1.tv_sec, tp1.tv_nsec);

	// chatter_pub.publish(msg);
	chatter_pub->publish(msg); // publish message
}
else if (count == -1) {
	bytedata = read_datafile(message_filename.c_str());
}

// Output publish_time [] to publish_time_ * byte.txt after evaluation
if (count == EVAL_NUM - 1) {
  if ((fp = fopen(output_filename.c_str(), "w")) != NULL) {
	  for (i=0; i < EVAL_NUM; i++) {

		  if (fprintf(fp, "%18.9lf\n", publish_time[i]) < 0) {
		    // Write error
		    printf("error : can't output publish_time.txt");
		    break;
		  }
	  }
	  fclose(fp);
	} else {
	  printf("error : can't output publish_time.txt");
	}

//	std::string out_f = "./evaluation/ack_time/ack_time_" + output_filename.substr(output_filename.find_last_of('_') + 1);;
//    if ((fp = fopen(out_f.c_str(), "w")) != NULL) {
//		for (i=0; i < EVAL_NUM; i++) {
//			if (fprintf(fp, "%18.9lf\n", ack_time[i]) < 0) {
//		    	// Write error
//		    	printf("error : can't output ack_time.txt");
//		    	break;
//			}
//		}
//		fclose(fp);
//	} else {
//	  	printf("error : can't output ack_time.txt");
//    }
	count = -2; // initilize for next date size
}
  
count++;
return 0;
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
  auto node = rclcpp::Node::make_shared("talker");
  
  // geting parameters from yaml file
  node->declare_parameter("QoS_Policy");
  int QoS_Policy_param;
  node->get_parameter_or("QoS_Policy", QoS_Policy_param, QoS_Policy);
  
  // QoS settings
  rmw_qos_profile_t custom_qos_profile;
  if( QoS_Policy_param == 1){
	custom_qos_profile = rmw_qos_profile_reliable;
  }
  else if( QoS_Policy_param == 2 ){
	custom_qos_profile = rmw_qos_profile_best_effort;
  }
  else if( QoS_Policy_param == 3){
	custom_qos_profile = rmw_qos_profile_history;
  }

  rclcpp::WallRate loop_rate(PUBLISH_Hz);
  bool ack_received = false;

    auto ack_sub = node->create_subscription<std_msgs::msg::String>(
    "acknowledgment", [&ack_received](const std_msgs::msg::String::SharedPtr msg) {
        printf("%d\n", msg->data == "OK");
        if (msg->data == "OK") {
            printf("ack recieved!");
            ack_received = true; // Set the flag to true when acknowledgment is received
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Acknowledgment received.");
        }
    }, custom_qos_profile);

  usleep(5000000);
  
  auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter", custom_qos_profile);

    // File names for each evaluation size
    std::vector<std::string> data_files = {
        "./evaluation/byte_data/data_256byte.txt",
        "./evaluation/byte_data/data_512byte.txt",
        "./evaluation/byte_data/data_1Kbyte.txt",
        "./evaluation/byte_data/data_2Kbyte.txt",
        "./evaluation/byte_data/data_4Kbyte.txt",
        "./evaluation/byte_data/data_8Kbyte.txt",
        "./evaluation/byte_data/data_16Kbyte.txt",
        "./evaluation/byte_data/data_32Kbyte.txt",
        "./evaluation/byte_data/data_64Kbyte.txt",
        "./evaluation/byte_data/data_128Kbyte.txt",
        "./evaluation/byte_data/data_256Kbyte.txt",
        "./evaluation/byte_data/data_512Kbyte.txt",
        "./evaluation/byte_data/data_1Mbyte.txt",
        "./evaluation/byte_data/data_2Mbyte.txt",
        "./evaluation/byte_data/data_4Mbyte.txt"
    };

    // Loop through each data size
    for (const auto &data_file : data_files) {
        printf("start evaluation %s \n", data_file.c_str());
		std::string data_size = data_file.substr(data_file.find_last_of('_') + 1);
        while (rclcpp::ok()) {
            eval_ros2(data_file, "./evaluation/publish_time/publish_time_" + data_size, chatter_pub);

            if (count == -1) {
                printf("end this data size evaluation \n");
                break;
            }

            // Reset acknowledgment flag and start timing
            ack_received = false;

            // Wait for acknowledgment
            while (!ack_received) {
                rclcpp::spin(node); // Process incoming messages
                loop_rate.sleep(); // Maintain the loop rate
            }

            	// Time recording
			if (clock_gettime(CLOCK_REALTIME,&tp2) < 0) {
	  			perror("clock_gettime begin");
	  			return 0;
  			}
            ack_time[count] = (double)tp2.tv_sec + (double)tp2.tv_nsec/ (double)1000000000L;


        }
        usleep(5000000); // Pause between evaluations
    }

    // Follow-through transactions for end messages
    count = 0;
    while (rclcpp::ok()) {
        auto msg = std::make_shared<std_msgs::msg::String>();
        std::stringstream ss;
        ss << "end" << count;
        msg->data = ss.str();
        chatter_pub->publish(msg);
        if (count++ == 100) {
            printf("---end evaluation---\n");
            break;
        }
        rclcpp::spin(node);
        loop_rate.sleep();
    }


    #pragma GCC diagnostic pop

    return 0;
}

