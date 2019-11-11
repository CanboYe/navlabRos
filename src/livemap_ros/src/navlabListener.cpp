#include "navlabListener.h"
#include "navlab.h"

void NavlabListener::on_failure(const mqtt::token &tok)
{
    std::cout << name_ << " failure";
    if (tok.get_message_id() != 0)
        std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
    std::cout << std::endl;
}

void NavlabListener::on_success(const mqtt::token &tok)
{
    std::cout << name_ << " success";
    if (tok.get_message_id() != 0)
        std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
    auto top = tok.get_topics();
    if (top && !top->empty())
        std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
    std::cout << std::endl;
}
/////////////////////////////////////////////////////////////////////////////

/**
 * Local callback & listener class for use with the client connection.
 * This is primarily intended to receive messages, but it will also monitor
 * the connection to the broker. If the connection is lost, it will attempt
 * to restore the connection and re-subscribe to the topic.
 */

// This deomonstrates manually reconnecting to the broker by calling
// connect() again. This is a possibility for an application that keeps
// a copy of it's original connect_options, or if the app wants to
// reconnect with different options.
// Another way this can be done manually, if using the same options, is
// to just call the async_client::reconnect() method.
void detectionCallback::reconnect()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    try
    {
        std::cout<<"reconnecting"<<std::endl;
        cli_->connect(connOpts_, nullptr, *this);
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << "Error: " << exc.what() << std::endl;
        exit(1);
    }
}

// Re-connection failure
void detectionCallback::on_failure(const mqtt::token &tok)
{
    std::cout << "Connection failed" << std::endl;
    if (++nretry_ > N_RETRY_ATTEMPTS)
        exit(1);
    reconnect();
}

// Re-connection success
// void detectionCallback::on_success(const mqtt::token &tok)
// {
//     std::cout << "\nConnection success" << std::endl;
//     std::cout << "\nSubscribing to topic '" << TOPIC << "'\n"
//               << "\tfor client " << CLIENT_ID
//               << " using QoS" << QOS << "\n"
//               << "\nPress Q<Enter> to quit\n"
//               << std::endl;

//     cli_->subscribe(TOPIC, QOS, nullptr, subListener_);
// }



// Callback for when the connection is lost.
// This will initiate the attempt to manually reconnect.
void detectionCallback::connection_lost(const std::string &cause)
{
    std::cout << "\nConnection lost" << std::endl;
    if (!cause.empty())
        std::cout << "\tcause: " << cause << std::endl;

    std::cout << "Reconnecting..." << std::endl;
    nretry_ = 0;
    reconnect();
}

// Callback for when a message arrives.
void detectionCallback::message_arrived(mqtt::const_message_ptr msg)
{
    std::cout << "Zone Detection Message arrived++++" << std::endl;
    std::string detectionJSON(msg->get_payload_str().begin(),
                               msg->get_payload_str().end());
    //std::cout << detectionJSON << std::endl;
    //std::string detectionJSON = msg->to_string();
    DetectionMessage dmessage;
    Utils::parseDetectionJSON(detectionJSON, dmessage);
    
    //std::cout << dmessage.ImageID_ << std::endl;
    // const std::string toDecode = dmessage.Image_;
    // dmessage.Image_ = Utils::base64_decode(toDecode);
    // std::vector<uchar> vectordata(dmessage.Image_.begin(),
    //                                     dmessage.Image_.end());
	// 	//Convert buffer to Mat
    // //std::cout << dmessage.Image_ << std::endl;
    // cv::Mat img = cv::imdecode(vectordata, CV_LOAD_IMAGE_COLOR);

    //saveImage and insert the hazard
    // m_Navlab->saveImage(img);

    //determine if the hazard is new
    if (!m_Navlab->selectHazard(dmessage))
    {
        m_Navlab->insertHazard(dmessage);
    }
    else
    {
        std::cout << "duplicated detection messages++++" << std::endl<< std::endl;
    }
    // std::cout << "Message arrived" << std::endl;
    // std::cout << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
    // std::cout << "\tpayload: '" << msg->to_string() << "'\n" << std::endl;
}

void detectionCallback::delivery_complete(mqtt::delivery_token_ptr token) 
{
    std::cout << "\tDelivery complete for token: "
    << (token ? token->get_message_id() : -1) << std::endl<< std::endl;
}