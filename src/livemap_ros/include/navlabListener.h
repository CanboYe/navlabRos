#ifndef NAVLAB_LISTENER_H
#define NAVLAB_LISTENER_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include "mqtt/async_client.h"
#include <opencv2/opencv.hpp>
#include "utils.h"

#include <fstream>
#include <string>
#include <iostream>
#include <pqxx/pqxx>

/////////////////////////////////////////////////////////////////////////////
//Listener//
/////////////////////////////////////////////////////////////////////////////
class Navlab;

class NavlabListener : public virtual mqtt::iaction_listener
{
public:
  NavlabListener(const std::string &name) : name_(name) {}
  //void listen();
  
private:
  std::string name_;

  void on_failure(const mqtt::token &tok) override;
  void on_success(const mqtt::token &tok) override;
};

/////////////////////////////////////////////////////////////////////////////
//Callbacks//
/////////////////////////////////////////////////////////////////////////////

/**
 * Local callback & listener class for use with the client connection.
 * This is primarily intended to receive messages, but it will also monitor
 * the connection to the broker. If the connection is lost, it will attempt
 * to restore the connection and re-subscribe to the topic.
 */
class detectionCallback : public virtual mqtt::callback,
                          public virtual mqtt::iaction_listener
{
private:
  // Counter for the number of connection retries
  int nretry_;
  // The MQTT client
  mqtt::async_client *cli_;
  // Options to use if we need to reconnect
  mqtt::connect_options &connOpts_;
  // An action listener to display the result of actions.
  NavlabListener subListener_;
  Navlab* m_Navlab;
  // This deomonstrates manually reconnecting to the broker by calling
  // connect() again. This is a possibility for an application that keeps
  // a copy of it's original connect_options, or if the app wants to
  // reconnect with different options.
  // Another way this can be done manually, if using the same options, is
  // to just call the async_client::reconnect() method.
  void reconnect();

  // Re-connection failure
  void on_failure(const mqtt::token &tok) override;

  // Re-connection success
//   void on_success(const mqtt::token &tok) override;

    // (Re)connection success
    // Either this or connected() can be used for callbacks.
    void on_success(const mqtt::token& tok) override {}

    // (Re)connection success
    void connected(const std::string& cause) override {
        std::cout << "\nConnection success" << std::endl;
        std::cout << "\nSubscribing to topic '" << TOPIC << "'\n"
            << "\tfor client " << CLIENT_ID
            << " using QoS" << QOS << "\n"
            << "\nPress Q<Enter> to quit\n" << std::endl;

        cli_->subscribe(TOPIC, QOS, nullptr, subListener_);
    }

  // Callback for when the connection is lost.
  // This will initiate the attempt to manually reconnect.
  void connection_lost(const std::string &cause) override;

  // Callback for when a message arrives.
  void message_arrived(mqtt::const_message_ptr msg) override;

  void delivery_complete(mqtt::delivery_token_ptr token) override;

  std::string SERVER_ADDRESS;
  std::string CLIENT_ID;
  std::string TOPIC;

  int QOS;
  int N_RETRY_ATTEMPTS;

public:
  detectionCallback(mqtt::async_client* cli, mqtt::connect_options &connOpts,
                    std::string &serveAddress, std::string &clientID, std::string &topic, int &qos, int &nRetryAttempts)
      : nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription"),
        SERVER_ADDRESS(serveAddress), CLIENT_ID(clientID), TOPIC(topic), QOS(qos), N_RETRY_ATTEMPTS(nRetryAttempts)
  {
  }
  void setNavlab(Navlab* Navlab){m_Navlab = Navlab;}
};

#endif //NAVLAB_LISTENER_H