/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <ros/ros.h>
#include <thread>
#include <vector>
#include <string>
#include <sstream>
#include <multi_uav/Drone.h>
#include <multi_uav/utils/GlobalPosition.h>
#include <multi_uav_se_mission/TypeParser.h>
#include <multi_uav_se_mission/CSerial.h>
#include <multi_uav_se_mission/Arrays.h>
#include <multi_uav_se_mission/SearcherStatistics.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <std_msgs/String.h>

#define MESSAGE_BLOCK_SIZE_BYTES 40

// statistics
int numberOfMessagesSent = 0;
int numberOfMessagesReceived = 0;
int numberOfIdentifiedTargets = 0;
int numberOfTargetsReceivedByWorkers = 0;
ros::Publisher statisticsStrPublisher;
ros::Publisher statisticsMessagesPublisher;

typedef struct Target {
  cv::Mat image;
  int type;
  bool isTargetReceivedByWorker;
  double lat;
  double lon;
  double alt;
  double yaw;

  std::string toString(){
    std::stringstream ss;
    ss << "type=" << type;
    ss << ";isTargetReceivedByWorker=" << isTargetReceivedByWorker;
    ss << ";lat=" << lat;
    ss << ";lon=" << lon;
    ss << ";alt=" << alt;
    ss << ";yaw=" << yaw;
    return ss.str();
  }

} Target;

std::vector<Target *> detectedTargets;

void print(std::string msgStr){
  std_msgs::String msg;
  msg.data = msgStr;
  statisticsStrPublisher.publish(msg);
  std::cout << msgStr << std::endl;
}

void printFrame(int uavId, std::string msg, std::vector<unsigned char> frame){

  std::stringstream ss;
  ss.precision(20);
  ss << "UAV " << uavId << "= " << msg << " ";
  for (int i = 0; i < frame.size(); i++) {
    int n = (int) multi_uav_se_mission::TypeParser::ucharToInt8t(frame[i]);
    ss << n << " ";
  }
  print(ss.str());

}

void publishStatisticMessage0(int searcherId, int objectId, int objectType, double lat, double lon, double alt, double yaw){

  std::stringstream ss;
  ss.precision(8);
  ss << "message0;";
  ss << ros::Time::now().toSec() << ";";
  ss << searcherId << ";";
  ss << objectId << ";";
  ss << objectType << ";";
  ss << lat << ";";
  ss << lon << ";";
  ss << alt << ";";
  ss << yaw << ";";

  std_msgs::String msg;
  msg.data = ss.str();
  statisticsMessagesPublisher.publish(msg);
}

void publishStatisticMessage1(int searcherId, int workerId, int objectId, int objectType, double distance, double minDistance){

  std::stringstream ss;
  ss.precision(8);
  ss << "message1;";
  ss << ros::Time::now().toSec() << ";";
  ss << searcherId << ";";
  ss << workerId << ";";
  ss << objectId << ";";
  ss << objectType << ";";
  ss << distance << ";";
  ss << minDistance << ";";

  std_msgs::String msg;
  msg.data = ss.str();
  statisticsMessagesPublisher.publish(msg);
}

void uavMission(multi_uav::Drone *d, double missionRadius, double missionStep, double missionAltitude){

  // creating a grid mission
  std::vector<std::vector<double>> missionGrid;

  missionGrid.push_back({-missionRadius, -missionRadius});

  double stepSignal = 1.0;
  for (double x = -missionRadius; x < missionRadius; x = x + missionStep) {
    for (double y = -missionRadius; y < missionRadius; y = y + missionStep) {

      missionGrid.push_back({0.0, missionStep * stepSignal});
    }
    missionGrid.push_back({missionStep, 0.0 * stepSignal});
    stepSignal = stepSignal * (-1.0);
  }

  std::stringstream ss;
  ss.precision(20);
  ss << "UAV " << d->parameters.id << "- Auto Grid Mission created with " << missionGrid.size() << " waypoints.";
  print(ss.str());

  // executing the mission
  d->configureToUseGlobalCoordinates();

  d->forceModeOffboard();

  d->arm();

  multi_uav::utils::GlobalPosition *gp = new multi_uav::utils::GlobalPosition(
    d->parameters.position.global.latitude,
    d->parameters.position.global.longitude,
    d->parameters.position.global.altitude,
    d->parameters.orientation.global.yaw
  );

  multi_uav::utils::GlobalPosition *gpHome = new multi_uav::utils::GlobalPosition(
    d->parameters.position.global.latitude,
    d->parameters.position.global.longitude,
    d->parameters.position.global.altitude,
    d->parameters.orientation.global.yaw
  );

  // adding the takeoff point
  gp->addMetersToAltitude(missionAltitude);
  gpHome->addMetersToAltitude(missionAltitude);

  std::stringstream ss1;
  ss1.precision(20);
  ss1 << "UAV " << d->parameters.id << "= going to position= {lat= " << gp->getLatitude() << "; lon= " << gp->getLongitude() << "; alt= " << gp->getAltitude() << "; yaw= " << gp->getYaw() << "}";
  print(ss1.str());

  d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

  for (int i = 0; i < missionGrid.size() && ros::ok(); i++) {

    gp->addPositionOffsetInMeters(missionGrid.at(i).at(0), missionGrid.at(i).at(1));

    std::stringstream ss2;
    ss2.precision(20);
    ss2 << "UAV " << d->parameters.id << "= going to position " << i << " of " << missionGrid.size() << "={lat= " << gp->getLatitude() << "; lon= " << gp->getLongitude() << "; alt= " << gp->getAltitude() << "; yaw= " << gp->getYaw() << "}";
    print(ss2.str());

    d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);
  }

  // go back to the home
  std::stringstream ss3;
  ss3.precision(20);
  ss3 << "UAV " << d->parameters.id << "= going back to home position {lat= " << gpHome->getLatitude() << "; lon= " << gpHome->getLongitude() << "; alt= " << gpHome->getAltitude() << "; yaw= " << gpHome->getYaw() << "}";
  print(ss3.str());

  d->goToGlobalPosition(gpHome->getLatitude(), gpHome->getLongitude(), gpHome->getAltitude(), gpHome->getYaw(), true);

  std::stringstream ss4;
  ss4.precision(20);
  ss4 << "UAV " << d->parameters.id << "= finishing the search mission.";
  print(ss4.str());

  d->~Drone();
}

bool isTargetInRadius(Target *t1, Target *t2, double minTargetRadius){

  double distancia = sqrt(pow((t2->lat)-(t1->lat), 2)+pow((t2->lon)-(t1->lon),2));
  if(distancia <= minTargetRadius) {
    return true;
  }
  else{
    return false;
  }
}

std::vector<int> detect(cv::Mat image){

  // get aruco dictionaries
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  // create a corners vector
  std::vector<std::vector<cv::Point2f> > corners;

  // create a detected markers id vector
  std::vector<int> ids;

  // detect markers on the image
  cv::aruco::detectMarkers(image, dictionary, corners, ids);

//  cv::Mat imageCopy;
//  image.copyTo(imageCopy);
//  // if at least one marker detected
//  if (ids.size() > 0) cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
//  cv::imshow("out", imageCopy);
//  char c=(char) cv::waitKey(25);
//  //if(c==27) break;


  return ids;
}

bool validateTarget(Target *t, double minTargetRadiusMeters){

  // se não detectou um objeto
  if(t->type == -1) return false;

  // checa a distância para os objetos já detectados
  // se a distância for maior que o raio definido armazena a target
  for(auto t2 : detectedTargets){
    if(isTargetInRadius(t, t2, minTargetRadiusMeters) && t->type == t2->type){
      return false;
    }
  }

  return true;
}

void uavDetection(multi_uav::Drone *d, double minTargetRadiusMeters){

  ros::Rate rate(10);
  while(ros::ok()){

    cv::Mat image;
    d->sensors.camera.rgb.image.copyTo(image);
    double lat = d->parameters.position.global.latitude;
    double lon = d->parameters.position.global.longitude;
    double alt = d->parameters.position.global.altitude;
    double yaw = d->parameters.orientation.global.yaw;
    bool isTargetSent = false;

    // try to detect a target
    std::vector<int> ids = detect(image);

    for (int i=0; i < ids.size(); i++) {

      //std::cout << "UAV " << d->parameters.id << ": target candidate type " << ids.at(i) << " detected." << std::endl;

      // create a candidate target
      Target *candidateTarget = new Target();

      candidateTarget->type = ids.at(i);
      candidateTarget->image = image;
      candidateTarget->lat = lat;
      candidateTarget->lon = lon;
      candidateTarget->alt = alt;
      candidateTarget->yaw = yaw;
      candidateTarget->isTargetReceivedByWorker = isTargetSent;

      // if the target is valid, then add to a detected targets array
      if(validateTarget(candidateTarget, minTargetRadiusMeters)){
        detectedTargets.push_back(candidateTarget);
        std::stringstream ss;
        ss.precision(20);
        ss << "UAV " << d->parameters.id << "= target detected " << (detectedTargets.size()-1) << "={type= " << candidateTarget->type << "; lat= " << candidateTarget->lat << "; lon= " << candidateTarget->lon << "; alt= " << candidateTarget->alt << "; yaw= " << candidateTarget->yaw << "}";
        print(ss.str());
        numberOfIdentifiedTargets++;
      }
    }

    rate.sleep();
  }

}

void rosLoop(){
  // ros loop
  ros::Rate rate(20);
  while (ros::ok()) {
    ros::spin();
    rate.sleep();
  }
}

void publishSearsherStatisticsLoop(){
  ros::NodeHandle nh;
  ros::Publisher statisticsPublisher = nh.advertise<multi_uav_se_mission::SearcherStatistics>("statistics", 1);
  ros::Publisher statisticsTargetArrayPublisher = nh.advertise<std_msgs::String>("statisticsTargets", 1);

  // ros loop
  ros::Rate rate(1);
  while (ros::ok()) {

    // numerical statistics
    multi_uav_se_mission::SearcherStatistics msg;
    msg.numberOfMessagesSent = numberOfMessagesSent;
    msg.numberOfMessagesReceived = numberOfMessagesReceived;
    msg.numberOfIdentifiedTargets = numberOfIdentifiedTargets;
    msg.numberOfTargetsReceivedByWorkers = numberOfTargetsReceivedByWorkers;
    statisticsPublisher.publish(msg);

    // publish task array
    std::stringstream ssTargets;
    for (int i=0; i<detectedTargets.size(); i++) {
      ssTargets << i << "=" << detectedTargets[i]->toString() << ";";
    }
    std_msgs::String msgTask;
    msgTask.data = ssTargets.str();
    statisticsTargetArrayPublisher.publish(msgTask);

    rate.sleep();
  }
}

void communicationSender(multi_uav_se_mission::CSerial * serial, int uavId){

  ros::Rate rate(1);
  while (ros::ok() && serial->isOpened()) {

    for (int i = 0; i < detectedTargets.size() && serial->isOpened(); i++) {

      if(detectedTargets.at(i)->isTargetReceivedByWorker == false){

        // 1 - Start frame
        ////////////////////////////
        std::vector<unsigned char> frameToSend;

        // start frame
        int8_t frameIdentification = -128;
        frameToSend.push_back(multi_uav_se_mission::TypeParser::int8tToUchar(frameIdentification));

        // message type
        int8_t messageType = 0;
        frameToSend.push_back(multi_uav_se_mission::TypeParser::int8tToUchar(messageType));

        // searcher id
        multi_uav_se_mission::Arrays::emplaceBack(frameToSend, multi_uav_se_mission::TypeParser::intToUcharArray(uavId));

        // object id
        multi_uav_se_mission::Arrays::emplaceBack(frameToSend, multi_uav_se_mission::TypeParser::intToUcharArray(i));

        // object type id
        multi_uav_se_mission::Arrays::emplaceBack(frameToSend, multi_uav_se_mission::TypeParser::intToUcharArray(detectedTargets.at(i)->type));

        // lat
        multi_uav_se_mission::Arrays::emplaceBack(frameToSend, multi_uav_se_mission::TypeParser::floatToUcharArray(detectedTargets.at(i)->lat));

        // long
        multi_uav_se_mission::Arrays::emplaceBack(frameToSend, multi_uav_se_mission::TypeParser::floatToUcharArray(detectedTargets.at(i)->lon));

        // alt
        multi_uav_se_mission::Arrays::emplaceBack(frameToSend, multi_uav_se_mission::TypeParser::floatToUcharArray(detectedTargets.at(i)->alt));

        // yaw
        multi_uav_se_mission::Arrays::emplaceBack(frameToSend, multi_uav_se_mission::TypeParser::floatToUcharArray(detectedTargets.at(i)->yaw));

        //send message
        serial->writeDataInBlocks(frameToSend);
        numberOfMessagesSent++;

        publishStatisticMessage0(uavId, i, detectedTargets.at(i)->type, detectedTargets.at(i)->lat, detectedTargets.at(i)->lon, detectedTargets.at(i)->alt, detectedTargets.at(i)->yaw);

        printFrame(uavId, "sending message", frameToSend);

        // sleep
        rate.sleep();

      }

    }

  }

  serial->~CSerial();

}

void communicationReceiver(multi_uav_se_mission::CSerial * serial, int uavId){

  ros::Rate rate(1);
  while (ros::ok() && serial->isOpened()) {

      // received frame
      ////////////////////////////
      std::vector<unsigned char> frame = serial->readDataBlock();

      printFrame(uavId, "receiving message", frame);

      // if frame size is different, ignore this frame
      if(frame.size() != MESSAGE_BLOCK_SIZE_BYTES) continue;

      // define a byte index counter
      int i = 0;

      // start frame byte
      int8_t frameIdentification = multi_uav_se_mission::TypeParser::ucharToInt8t(frame[i]);
      i += multi_uav_se_mission::TypeParser::SIZE_INT8_T_BYTES;

      if((int) frameIdentification != -128) continue;

      // message type
      int8_t messageType = multi_uav_se_mission::TypeParser::ucharToInt8t(frame[i]);
      i += multi_uav_se_mission::TypeParser::SIZE_INT8_T_BYTES;

      if((int) messageType != 1) continue;

      // searcher id
      int searcherId = multi_uav_se_mission::TypeParser::ucharArrayToInt(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_INT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_INT_BYTES;

      // worker id
      int workerId = multi_uav_se_mission::TypeParser::ucharArrayToInt(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_INT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_INT_BYTES;

      // object id
      int objectId = multi_uav_se_mission::TypeParser::ucharArrayToInt(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_INT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_INT_BYTES;

      // object type
      int objectType = multi_uav_se_mission::TypeParser::ucharArrayToInt(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_INT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_INT_BYTES;

      // distance
      float distance = multi_uav_se_mission::TypeParser::ucharArrayToFloat(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_FLOAT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_FLOAT_BYTES;

      publishStatisticMessage1(searcherId, workerId, objectId, objectType, distance, 0.0);

      numberOfMessagesReceived++;

      // if searcher id == uav id and detected object contains
      if(searcherId == uavId && (detectedTargets.size() > objectId)){

        // if target types matches
        if (detectedTargets.at(objectId)->type == objectType) {
          detectedTargets.at(objectId)->isTargetReceivedByWorker = true;
          std::stringstream ss;
          ss.precision(20);
          ss << "UAV " << uavId << "= target " << objectId << " received by worker " << workerId << ".";
          print(ss.str());
          numberOfTargetsReceivedByWorkers++;
        }

      }


      // sleep
      rate.sleep();

  }

  serial->~CSerial();

}

int main(int argc, char **argv){
  ros::init(argc, argv, "searcher_node");
  ros::NodeHandle nh;

  statisticsStrPublisher = nh.advertise<std_msgs::String>("statisticsLog", 1);
  statisticsMessagesPublisher = nh.advertise<std_msgs::String>("statisticsMessages", 1);

  // configuring cout precision
  std::cout.precision(20);

  //parameters
  int uavId = 0;
  double missionRadius = 10.0;
  double missionStep = 1.0;
  double missionAltitude = 2.0;
  double minTargetRadiusMeters = 2.0;
  std::string serialPort = "";
  int baud = 9600;

  //get parameters
  std::stringstream ss;
  ss.precision(20);
  if(nh.hasParam(ros::this_node::getName() + "/uavId")){
    nh.getParam(ros::this_node::getName() + "/uavId", uavId);
  }
  else {
    ss << "Unable to get uavId parameter.";
    print(ss.str());
    return 0;
  }
  if(nh.hasParam(ros::this_node::getName() + "/minTargetRadiusMeters")){
    nh.getParam(ros::this_node::getName() + "/minTargetRadiusMeters", minTargetRadiusMeters);
  }
  else {
    ss << "UAV " << uavId << "= Unable to get minTargetRadiusMeters parameter.";
    print(ss.str());
    return 0;
  }
  if(nh.hasParam(ros::this_node::getName() + "/missionRadius")){
    nh.getParam(ros::this_node::getName() + "/missionRadius", missionRadius);
  }
  else {
    ss << "UAV " << uavId << "= Unable to get missionRadius parameter.";
    print(ss.str());
    return 0;
  }
  if(nh.hasParam(ros::this_node::getName() + "/missionStep")){
    nh.getParam(ros::this_node::getName() + "/missionStep", missionStep);
  }
  else {
    ss << "UAV " << uavId << "= Unable to get missionStep parameter.";
    print(ss.str());
    return 0;
  }
  if(nh.hasParam(ros::this_node::getName() + "/missionAltitude")){
    nh.getParam(ros::this_node::getName() + "/missionAltitude", missionAltitude);
  }
  else {
    ss << "UAV " << uavId << "= Unable to get missionAltitude parameter.";
    print(ss.str());
    return 0;
  }
  if(nh.hasParam(ros::this_node::getName() + "/serialPort")){
    nh.getParam(ros::this_node::getName() + "/serialPort", serialPort);
  }
  else {
    ss << "UAV " << uavId << "= Unable to get serialPort parameter.";
    print(ss.str());
    return 0;
  }
  if(nh.hasParam(ros::this_node::getName() + "/baud")){
    nh.getParam(ros::this_node::getName() + "/baud", baud);
  }
  else {
    ss << "UAV " << uavId << "= Unable to get baud parameter.";
    print(ss.str());
    return 0;
  }

  multi_uav_se_mission::CSerial *serial = new multi_uav_se_mission::CSerial();

  std::stringstream ss1;
  ss1.precision(20);
  ss1 << "UAV " << uavId << "= Connecting on serialPort = " << serialPort << " baud = " << baud;
  print(ss1.str());

  if(serial->openPort(serialPort, baud)){

    serial->setMessageBlockSize(MESSAGE_BLOCK_SIZE_BYTES);

    multi_uav::Drone *d = new multi_uav::Drone(nh, uavId, false);

    std::thread rosLoopThread(&rosLoop);
    std::thread publishSearsherStatisticsLoopThread(&publishSearsherStatisticsLoop);
    std::thread uavMissionThread(&uavMission, d, missionRadius, missionStep, missionAltitude);
    std::thread uavDetectionThread(&uavDetection, d, minTargetRadiusMeters);
    std::thread communicationSenderThread(&communicationSender, serial, uavId);
    std::thread communicationReceiverThread(&communicationReceiver, serial, uavId);

    rosLoopThread.join();
    publishSearsherStatisticsLoopThread.join();
    uavMissionThread.join();
    uavDetectionThread.join();
    communicationSenderThread.join();
    communicationReceiverThread.join();

  }
  else{
    std::stringstream ss2;
    ss2.precision(20);
    ss2 << "UAV " << uavId << "= Could not connect to serial port!";
    print(ss2.str());
  }

  serial->~CSerial();

  return 0;

}
