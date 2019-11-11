#ifndef NAVLAB_H
#define NAVLAB_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>

#include <cstdlib>
#include <thread>	// For sleep
#include <atomic>
#include <chrono>
#include <cstring>
#include "mqtt/async_client.h"
#include <opencv2/opencv.hpp>

#include <fstream>
#include <streambuf>
#include <ctime>

#include "utils.h"
#include "settings.h"
#include <pqxx/pqxx>

// const std::string DFLT_SERVER_ADDRESS	{ "tcp://squall.elijah.cs.cmu.edu:1883" };
// const std::string DFLT_CLIENT_ID		{ "async_publish" };

// const std::string TOPIC { "HAZARDS_DETECTED" };

// const char* PAYLOAD1 = "Hello World!";
// const char* PAYLOAD2 = "Hi there!";
// const char* PAYLOAD3 = "Is anyone listening?";
// const char* PAYLOAD4 = "Someone is always listening.";

// const char* LWT_PAYLOAD = "Last will and testament.";

// const int  QOS = 1;

// const auto TIMEOUT = std::chrono::seconds(10);

using namespace std;

class Navlab{
    public:
        Navlab(ros::NodeHandle &n);
        ~Navlab();
        void rawImageCallback(const sensor_msgs::ImageConstPtr &msg);
        void detectionImageCallback(const sensor_msgs::ImageConstPtr &msg);
        void navlabGPSCallback(const sensor_msgs::NavSatFixConstPtr &fix);
        void runROS();
        void logGPS(const double &latitude , const double &longitude);

        //void makeDetectionJSON(std::string &sendMessage, DetectionMessage &message);

        void sendDetectionToZoneCloudlet(DetectionMessage &message); //TODO
        void writeToVideo();

        double measureDistance(double lat1, double lon1, double lat2, double lon2);
        int insertHazard(DetectionMessage &msg);
        int selectHazard(DetectionMessage &msg);
        double get_dLat(double lat1, double lon1, double distance);
        double get_dLon(double lat1, double lon1, double distance);

    private:
        //ROS member variables
        ros::NodeHandle m_nodeHandle;
        ros::Rate m_loop_rate;
        ros::Subscriber m_rawImageSub;
        ros::Subscriber m_detectedImageSub;
        ros::Subscriber m_navlabGPSSub;
        //
        int m_numRawImages;
        //cv::VideoWriter m_videoWriter;
        std::string m_videoFile;
        int m_fourcc;
        int m_fps;
        int m_numVideos;
        // 
        double m_navLat; 
        double m_navLong;
        double m_oldLat;
        double m_oldLong;
        double m_oldAngle;
        double m_navAngle;
        double m_distFromLastSend;
        double m_LatLastDetection;
        double m_LongLastDetection;
        //
        cv::Mat* m_lastDetection;
        //
        int m_numDetections;
        //
        std::string m_workingDirectory;
        std::string m_frameDirectory;
        std::string m_detectionsDirectory;
        std::string m_videoDirectory;
        std::string m_HAZARD_TOPIC;
        std::string m_DRIVE_TOPIC;
        //
        const std::string OPENCV_WINDOW; 
        const std::string OPENCV_WINDOW2; 
        const std::string OPENCV_WINDOW3;
        //
        const std::string m_nameGPSLog;
        std::ofstream m_GPSFile;
        int m_numGPSCoords;

        mqtt::async_client* m_client;
        // callback m_cb;
        mqtt::connect_options m_conopts;
        mqtt::token_ptr m_conntok;
        
        clock_t m_startTime; 

        // Database related
        DatabaseContainer m_HazardDB;
        int m_numInsertions;
        int m_numReceived;
        


};

// /**
//  * A base action listener.
//  */
// class action_listener : public virtual mqtt::iaction_listener
// {
// protected:
// 	void on_failure(const mqtt::token& tok) override {
// 		cout << "\tListener failure for token: "
// 			<< tok.get_message_id() << endl;
// 	}

// 	void on_success(const mqtt::token& tok) override {
// 		cout << "\tListener success for token: "
// 			<< tok.get_message_id() << endl;
// 	}
// };

// class action_listener : public virtual mqtt::iaction_listener
// {
// 	std::string name_;

// 	void on_failure(const mqtt::token& tok) override {
// 		std::cout << name_ << " failure";
// 		if (tok.get_message_id() != 0)
// 			std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
// 		std::cout << std::endl;
// 	}

// 	void on_success(const mqtt::token& tok) override {
// 		std::cout << name_ << " success";
// 		if (tok.get_message_id() != 0)
// 			std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
// 		auto top = tok.get_topics();
// 		if (top && !top->empty())
// 			std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
// 		std::cout << std::endl;
// 	}

// public:
// 	action_listener(const std::string& name) : name_(name) {}
// };
// /////////////////////////////////////////////////////////////////////////////

// class callback : public virtual mqtt::callback
// {
// public:
// 	void connection_lost(const string& cause) override {
// 		cout << "\nConnection lost" << endl;
// 		if (!cause.empty())
// 			cout << "\tcause: " << cause << endl;
// 	}

// 	void delivery_complete(mqtt::delivery_token_ptr tok) override {
// 		cout << "\tDelivery complete for token: "
// 			<< (tok ? tok->get_message_id() : -1) << endl;
// 	}
// };

// class callback : public virtual mqtt::callback,
//                           public virtual mqtt::iaction_listener
// {
// private:
//     // Counter for the number of connection retries
//     int nretry_;
//     // The MQTT client
//     mqtt::async_client &cli_;
//     // Options to use if we need to reconnect
//     mqtt::connect_options &connOpts_;
//     // An action listener to display the result of actions.
//     action_listener subListener_;
//     Navlab* m_navlab;
//     // This deomonstrates manually reconnecting to the broker by calling
//     // connect() again. This is a possibility for an application that keeps
//     // a copy of it's original connect_options, or if the app wants to
//     // reconnect with different options.
//     // Another way this can be done manually, if using the same options, is
//     // to just call the async_client::reconnect() method.
//     void reconnect() {
//         std::this_thread::sleep_for(std::chrono::milliseconds(2500));
//         try {
//             cli_.connect(connOpts_, nullptr, *this);
//         }
//         catch (const mqtt::exception& exc) {
//             std::cerr << "Error: " << exc.what() << std::endl;
//             exit(1);
//         }
//     }

//     // Re-connection failure
//     void on_failure(const mqtt::token& tok) override {
//         std::cout << "Connection attempt failed" << std::endl;
//         if (++nretry_ > N_RETRY_ATTEMPTS)
//             exit(1);
//         reconnect();
//     }

//     // (Re)connection success
//     // Either this or connected() can be used for callbacks.
//     void on_success(const mqtt::token& tok) override {}

//     // (Re)connection success
//     void connected(const std::string& cause) override {
//         std::cout << "\nConnection success" << std::endl;
//         std::cout << "\nSubscribing to topic '" << TOPIC << "'\n"
//             << "\tfor client " << CLIENT_ID
//             << " using QoS" << QOS << "\n"
//             << "\nPress Q<Enter> to quit\n" << std::endl;

//         cli_.subscribe(TOPIC, QOS, nullptr, subListener_);
//     }

//     // Callback for when the connection is lost.
//     // This will initiate the attempt to manually reconnect.
//     void connection_lost(const std::string& cause) override {
//         std::cout << "\nConnection lost" << std::endl;
//         if (!cause.empty())
//             std::cout << "\tcause: " << cause << std::endl;

//         std::cout << "Reconnecting..." << std::endl;
//         nretry_ = 0;
//         reconnect();
//     }

//     // Callback for when a message arrives.
//     void message_arrived(mqtt::const_message_ptr msg) override;

//     void delivery_complete(mqtt::delivery_token_ptr token) override {}

//     std::string SERVER_ADDRESS;
//     std::string CLIENT_ID;
//     std::string TOPIC;

//     int QOS;
//     int N_RETRY_ATTEMPTS;

// public:
//     callback(mqtt::async_client &cli, mqtt::connect_options &connOpts,
//                         std::string &serveAddress, std::string &clientID, std::string &topic, int &qos, int &nRetryAttempts)
//         : nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription"),
//             SERVER_ADDRESS(serveAddress), CLIENT_ID(clientID), TOPIC(topic), QOS(qos), N_RETRY_ATTEMPTS(nRetryAttempts)
//     {
//     }
//     void setNavlab(Navlab* navlab){m_navlab = navlab;}
// };

// /////////////////////////////////////////////////////////////////////////////

// /**
//  * A derived action listener for publish events.
//  */
// class delivery_action_listener : public action_listener
// {
// 	atomic<bool> done_;

// 	void on_failure(const mqtt::token& tok) override {
// 		action_listener::on_failure(tok);
// 		done_ = true;
// 	}

// 	void on_success(const mqtt::token& tok) override {
// 		action_listener::on_success(tok);
// 		done_ = true;
// 	}

// public:
// 	delivery_action_listener() : done_(false) {}
// 	bool is_done() const { return done_; }
// };

#endif //NAVLAB_H