/**************************************** *************************
* Filename: TAhelper.cpp
* Authors: Weiqi XU, wex064@eng.ucsd.edu
*		   Yiding QIU, yiqiu@eng.ucsd.edu
* Date: 03/17/2019
* Final Project: TAhelper - an Autonomous Student Self Check-in 
*				 and Homework Delivery Robot
*
*Description: This is the final project for CSE 276B.  
*			  It demonstrates a robot that check students' attendance, 
*			  collect homework and deliver homework to the TA. 
*			  This program subscribes to the usb_cam node, blob topic, 
*			  face_recognition topic, turtlebot_navigation topic, 
*			  and uses sound_play package.
			
*How to use:
 Usage:
*	roscore
*	roslaunch turtlebot_bringup minimal.launch
*
*   rosrun sound_play soundplay_node.py

*	roscd usb_cam
*	rosparam set usb_cam/pixel_format yuyv 
*	rosrun usb_cam usb_cam_node 
*	rosrun image_view image_view image:=/usb_cam/image_raw
*	roslaunch cmvision colorgui image:=/camera/rgb/image_raw
*	<this is for color calibration, close when done>
*	roslaunch cmvision cmvision.launch image:=/usb_cam/image_raw
*	<cnrl-c to kill process>
*	rosparam set /cmvision/color_file ~/turtlebot_ws/src/cmvision/colors.txt
*	rosrun cmvision cmvision image:=/usb_cam/image_raw
*	
*	roscd face_recognition
*	rosrun face_recognition Fserver
*
*	export TURTLEBOT_MAP_FILE=/tmp/my_map.yaml
*	roslaunch turtlebot_navigation amcl_demo.launch
*
*	rosrun TAhelper TAhelper
*******************************************************************/

#include <kobuki_msgs/BumperEvent.h>
#include <ros/ros.h>
#include <cmvision/Blobs.h>
#include <stdio.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>
#include <fstream>
#include <string>
#include <unistd.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <face_recognition/FRClientGoal.h>
#include <face_recognition/FaceRecognitionAction.h>
#include <signal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <boost/thread.hpp>

using namespace std;
ros::Publisher pub;
face_recognition::FaceRecognitionGoal face_goal; //Goal message for face_recognition
actionlib::SimpleActionClient<face_recognition::FaceRecognitionAction> * face_ac; //action lib client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; 

/* define global variables and initialization */
bool got_goal_blobs = false;
bool arrive_at_goal = false;
int system(const char *command);
std::string command;
bool start_clock = false;
bool task1_finished = false;
bool detect_TA = false;
bool detect_passenger = false;
bool detect_student  = false;
std::string detected_student; 
bool start_ask_for_help = false;


/************************************************************
 * Function Name: display
 * Parameters: path to images which need to show
 * Returns: void
 * Description: This is the function that display certain interface.
 ***********************************************************/
void display (const char* path){  
	while(true){
	    cv::Mat image;
	    image = cv::imread(path, CV_LOAD_IMAGE_COLOR);  // Read the file
	    // Check for invalid input
	    if(! image.data )   
	        std::cout <<  "Could not open or find the image" << std::endl;

	    cv::namedWindow( "Display window", CV_WINDOW_NORMAL);// Create a window for display.
	    cv::resizeWindow("Display window", 1920,1080);
	    imshow( "Display window", image ); // Show our image inside the window.
	    int key = (cv::waitKey(0) & 0xFF);
	    if (key == 'q') break;
	}

}

/************************************************************
 * Function Name: frclientCallback
 * Parameters: const face_recognition::FRClientGoalConstPtr& msg
 * Returns: void
 * Description: This is the callback function of the face_recognition topic. 
 *				Called everytime FRClientGoal message is received. 
 *				Then it processes each message and sends the corresponding goal 
 *				to the server and registers feedback and result call back 
 *				functions.
 ***********************************************************/

void frclientCallback(const face_recognition::FRClientGoalConstPtr& msg)
  {
     
     ROS_INFO("request for sending goal [%i] is received", msg->order_id);
     face_goal.order_id = msg->order_id;
     face_goal.order_argument = msg->order_argument;
     face_ac->sendGoal(face_goal, &doneCb, &activeCb, &feedbackCb);   

  }
// Called once when the goal sent to fFserver completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const face_recognition::FaceRecognitionResultConstPtr& result)
{

}
// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback (people detected) is received for the goal
void feedbackCb(const face_recognition::FaceRecognitionFeedbackConstPtr& feedback)
{
  	if(feedback->order_id == 1 ){
  		// A student is considered as detected when the confidence reach the threshold
		if (feedback->confidence[0] > 0.8){
			detect_student = true;
			detected_student = feedback->names[0].c_str();
			cout << detected_student <<endl;
			// condition for "TA" Cassie is detected
			if (detected_student == "cassie" && feedback->confidence[0] > 0.9){
				detect_TA = true;
				ROS_INFO("detect cassie");
			}
		}
		// condition for a passenger is detected    	
		if (feedback->confidence[0] > 0.3)
   	 		detect_passenger = true;
  	}
}

// The exit_handler
void exit_handler(int s)
{
  delete(face_ac);
  ros::shutdown();
}

/************************************************************
 * Function Name: blobsCallBack
 * Parameters: const cmvision::Blobs
 * Returns: void
 * Description: This is the callback function of the /blobs topic;
 *				send feedback when certain color blob (yellow) is detected.
 ***********************************************************/

void blobsCallBack (const cmvision::Blobs& blobsIn)
{
    double goal_area = 0; 
    double goal_centroid_x = 0;
    double goal_centroid_y = 0;
    int goal_blobs_count = 0; 

    if (blobsIn.blob_count > 0){
		for (int i = 0; i < blobsIn.blob_count; i++){
   			/* if yellow color blob is detected, add up the centroid coordinates and blob area. */
		    if (blobsIn.blobs[i].red == 255 && blobsIn.blobs[i].green == 255 && blobsIn.blobs[i].blue == 0){
                goal_blobs_count += 1;
		      	goal_centroid_x += blobsIn.blobs[i].x;
		      	goal_centroid_y += blobsIn.blobs[i].y;
                goal_area += blobsIn.blobs[i].area;
		    }		
		}
        // if the goal area is larger than a threshold, a homework is detected.
        if (goal_area > 200){
            got_goal_blobs = true;
            ROS_INFO("Got homework");
        }

	}
}

/************************************************************
 * Function Name: Keyboard
 * Parameters: none
 * Returns: keyboard input
 * Description: This is the function that receives keyboard input.
 ***********************************************************/
int Keyboard(){
	bool no_keyboard_input = true;
	int input;
	while(no_keyboard_input){
		int key1 = (cv::waitKey(1) & 0xFF);
		if (key1 == 'y'){
		    input = 1;
		    no_keyboard_input = false;
		    }
		else if (key1 == 'n'){
		    input = 0;
		    no_keyboard_input = false;
		  }
	}
  	return input;
}

/************************************************************
 * Main function
 * Description: Subscribes to the blob topic, face_recognition topic, 
 *              usb_cam node, turtlebot_navigation topic, 
 *			    and uses sound_play package.
 * Function:	Recognize students to check their attendances, and whether he/she 
 *				brings the homework or not. After all students have checked in,  
 *              deliver the homework to TA. If can't find TA in the office, ask
 *				passengers for help.
 ***********************************************************/

int main (int argc, char** argv)
{
  // Initialize ROS
	ros::init (argc, argv, "blob");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1);
	//subscribe to /blobs topic
	ros::Subscriber blobsSubscriber = nh.subscribe("/blobs", 1, blobsCallBack);
    //face_recognition send goal action 
	face_ac = new actionlib::SimpleActionClient<face_recognition::FaceRecognitionAction>("face_recognition", true);
	//wait for the server 
  	face_ac->waitForServer();
  	//subscribe to the /face_recogntion/FRClientGoal topic 
  	ros::Subscriber FaceSubscriber = nh.subscribe("fr_order", 1, frclientCallback);
  	//send goal to face_recognition Fserver to recognize people continuously
	system("rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 1 'none'");
	sleep(5);
	//initialize looping variable
	int i = 0;
	int j = 0;
 
  	// Start collecting homework
  	boost::thread t(display, "/home/turtlebot/Desktop/background/check_in.jpeg");
  	sleep(2);
	system("rosrun sound_play say.py 'hi, I will start check in and collect your homework!'");
	sleep(3);
	// when haven't finish checking students' attendances
 	while (task1_finished == false){
 		//detect students
	    for(i=0; i<8; i++){
	      ros::spinOnce();
	      sleep(0.5);
	    }
	    if (detect_student == true) {
			start_clock = false;
			ROS_INFO("Student checked in");
			command = "rosrun sound_play say.py 'hi," + detected_student + ", please turn in your homework.'";
			system(command.c_str());
			sleep(5);
	        // detect homework
	        for (int i=0; i<7; ++i){
				ros::spinOnce();
				if (got_goal_blobs){
				    //if student's homework is detected
				    command = "rosrun sound_play say.py 'thank you'";
				    system(command.c_str());
				  	sleep(2);
				    break;
				}
				// if no homework is detected, remind him/her to turn in homework with a louder voice
				else if (i==3){
				    command = "rosrun sound_play say.py 'nothing to turn in today?'";
				    system(command.c_str());
				    sleep(2);
				}
			}
			// if the student still didn't turn in, proceed to next
			if (got_goal_blobs == false){
				command = "rosrun sound_play say.py 'Ok, next'";
				system(command.c_str());
				sleep(4); // wait for student to go away
			}
          	// reset the parameters
            got_goal_blobs = false;
            detect_student = false;
			detected_student = " ";
	    }
	    // if no student show up, start counting
	    else{
			cout << j << endl;
	        if (start_clock == false){
		        start_clock = true;
				j = 0;
	    	}
	        else {
				j += 1;
				sleep(1);
				// after a while, ask if anyone else want to check in 
			    if (j == 8)
			    	system("rosrun sound_play say.py 'anyone else want to check in?'");
			    // no student want to check in anymore, move to delivery task
			    else if (j == 13){
					start_clock = false;
					task1_finished = true;
					detect_TA = false;
					break;
	        	}
			}
	    }
 	}
 	// after the check-in finishes, start homework delivery
	sleep(3);
	ROS_INFO("start SLAM");
	system("rosrun sound_play say.py 'start delivery'");
	//SLAM
	sleep(2);
	MoveBaseClient ac("move_base", true);
   	
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
       ROS_INFO("Waiting for the move_base action server to come up");
    }
  
	move_base_msgs::MoveBaseGoal goal;

	//we'll send a goal to the robot to head to office
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	// send office location, and turn 90 degrees clockwise after arrived
	goal.target_pose.pose.position.x = 5.4;
	goal.target_pose.pose.position.y = 1.99;
	goal.target_pose.pose.orientation.z = 0.7;
	goal.target_pose.pose.orientation.w = -0.7;
	
    ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	// Wait for the action to return
	ac.waitForResult();
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("You have reached the goal!");
	else
		ROS_INFO("The base failed for some reason");
	sleep(4);
	detect_TA = false;

	while(ros::ok()){

		//arrive at office, start looking for TA
		for (i=0; i<8; i++){
			ros::spinOnce;
			//spin to look around
			sleep(0.5);
		}
		sleep(1);
		//if TA is at office, deliver the homework right away, then terminate
		if (detect_TA == true){
			boost::thread t3(display, "/home/turtlebot/Desktop/background/find_TA_frog.jpeg");
			ROS_INFO("detect TA");
			sleep(3);
			system("rosrun sound_play say.py 'Hi, cassie. I checked students attendencies and bring the homework.'");
			sleep(4);
			return 0;
		}
		// otherwise, turn around (180 degrees) and wait for passenger's help
		else{
			// turn around
			goal.target_pose.header.stamp = ros::Time::now();
	    	goal.target_pose.pose.orientation.z = 1;
			goal.target_pose.pose.orientation.w = 0;
			
		    ROS_INFO("Sending goal");
			ac.sendGoal(goal);
			// Wait for the action to return
			ac.waitForResult();
			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO("You have reached the goal!");
			else
			ROS_INFO("The base failed for some reason");

			sleep(3);
			detect_TA = false;
			//show sad face 
			boost::thread t1(display, "/home/turtlebot/Desktop/background/ask_1_frog.jpg");
			sleep(3);

			// while TA is not found, looking for passengers for help
			while (detect_TA == false){
			
				for (i=0; i< 5; i++)
					ros::spinOnce();
				if (detect_passenger == true){
					// find passenger, say: Excuse me, could you please help me a little bit?
					system("rosrun sound_play play.py /home/turtlebot/Desktop/sound_clips/1_m.wav");
					sleep(3); // wait for people to approach.
					detect_passenger = false;
					for (i=0; i<4; i++){ 
						sleep(1);
						ros::spinOnce();	
					}
					// if people stop rather than ignore it
					if (detect_passenger == true){ 
						// ask if they know where "TA" Cassie is. If they know, press y, otherwise press n.
						system("rosrun sound_play play.py /home/turtlebot/Desktop/sound_clips/1.wav");
						sleep(6);
						system("rosrun sound_play play.py /home/turtlebot/Desktop/sound_clips/2.wav");
						sleep(2);
						//wait for keyboard input
	         			int ans = Keyboard();
	         			// if passenger knows, ask him/she to show robot the way
						if (ans == 1){
							boost::thread t2(display, "/home/turtlebot/Desktop/background/ask_2_frog.jpg");
							sleep(2);
							system("rosrun sound_play play.py /home/turtlebot/Desktop/sound_clips/3_m.wav");
	            			sleep(2);
							// get keyboard input
							int ans2 = Keyboard();
	              			if (ans2 == 1){
								//start following
								boost::thread t3(display, "/home/turtlebot/Desktop/background/arrive_frog.jpeg");
								sleep(1);
								// start follower mode
								system("roslaunch turtlebot_follower follower.launch");
								//after arrive at the place, look for TA again
								for (i=0; i<5;i++){
									ros::spinOnce();
									sleep(0.5);
								}
								if (detect_TA == true){
									boost::thread t4(display, "/home/turtlebot/Desktop/background/find_TA_frog.jpeg");
									ROS_INFO("detect TA");
									sleep(1);
									system("rosrun sound_play play.py /home/turtlebot/Desktop/sound_clips/4_m.wav");
									sleep(10);
									return 0;
								}
								//if TA is not there, reset parameters and wait for others to help
								else{
									detect_passenger = false;
									detect_TA = false;
									sleep(5); //waiting for the passenger to go away
								}
							}
							// if passenger refuses to help, reset parameters and wait for others to help
							else{
								system("rosrun sound_play play.py /home/turtlebot/Desktop/sound_clips/ok_m.wav");
								detect_passenger = false;
								detect_TA = false;
								sleep(5); // wait for people to walk away
							}
						}
						//if passenger don't know where TA is, reset parameters and wait for others to help
						else{
							system("rosrun sound_play play.py /home/turtlebot/Desktop/sound_clips/ok_m.wav");
							detect_passenger = false;
							detect_TA = false;						
							sleep(5); // wait for people to walk away
						}
					}
				} 
			}
		}
		loop_rate.sleep();
	}
}

