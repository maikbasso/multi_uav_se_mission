/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <ros/ros.h>
#include <thread>
#include <vector>
#include <string>
#include <multi_uav/Drone.h>
#include <multi_uav/utils/GlobalPosition.h>
#include <multi_uav_se_mission/TypeParser.h>
#include <multi_uav_se_mission/CSerial.h>
#include <multi_uav_se_mission/Arrays.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#define MESSAGE_BLOCK_SIZE_BYTES 40

typedef struct Target {
  cv::Mat image;
  int type;
  bool isTargetReceivedByWorker;
  double lat;
  double lon;
  double alt;
  double yaw;
} Target;

std::vector<Target *> detectedTargets;

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

  std::cout << "UAV " << d->parameters.id << ": Auto Grid Mission created with " << missionGrid.size() << " waypoints." << std::endl;

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
  );;

  // adding the takeoff point
  gp->addMetersToAltitude(missionAltitude);
  gpHome->addMetersToAltitude(missionAltitude);

  std::cout << "UAV " << d->parameters.id << ": going to position: {lat: " << gp->getLatitude() << ", lon: " << gp->getLongitude() << ", alt: " << gp->getAltitude() << ", yaw: " << gp->getYaw() << "}" << std::endl;

  d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

  for (int i = 0; i < missionGrid.size() && ros::ok(); i++) {

    gp->addPositionOffsetInMeters(missionGrid.at(i).at(0), missionGrid.at(i).at(1));

    std::cout << "UAV " << d->parameters.id << ": going to position " << i << " of " << missionGrid.size() << ":{lat: " << gp->getLatitude() << ", lon: " << gp->getLongitude() << ", alt: " << gp->getAltitude() << ", yaw: " << gp->getYaw() << "}" << std::endl;

    d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);
  }

  // go back to the home
  std::cout << "UAV " << d->parameters.id << ": going back to home position {lat: " << gpHome->getLatitude() << ", lon: " << gpHome->getLongitude() << ", alt: " << gpHome->getAltitude() << ", yaw: " << gpHome->getYaw() << "}" << std::endl;

  d->goToGlobalPosition(gpHome->getLatitude(), gpHome->getLongitude(), gpHome->getAltitude(), gpHome->getYaw(), true);

  std::cout << "UAV " << d->parameters.id << ": finishing the search mission." << std::endl;

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
        std::cout << "UAV " << d->parameters.id << ": target detected " << (detectedTargets.size()-1) << ":{type: " << candidateTarget->type << ", lat: " << candidateTarget->lat << ", lon: " << candidateTarget->lon << ", alt: " << candidateTarget->alt << ", yaw: " << candidateTarget->yaw << "}" << std::endl;
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

void printFrame(int uavId, std::string msg, std::vector<unsigned char> frame){

  std::cout << "UAV " << uavId << ": " << msg << " ";
  for (int i = 0; i < frame.size(); i++) {
    int n = (int) multi_uav_se_mission::TypeParser::ucharToInt8t(frame[i]);
    std::cout << n << " ";
  }
  std::cout << std::endl;

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

      if(frameIdentification != (int8_t) -128) continue;

      // message type
      int8_t messageType = multi_uav_se_mission::TypeParser::ucharToInt8t(frame[i]);
      i += multi_uav_se_mission::TypeParser::SIZE_INT8_T_BYTES;

      if(messageType != (int8_t) 1) continue;

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


      // if searcher id == uav id and detected object contains
      if(searcherId == uavId && (detectedTargets.size() > objectId)){

        // if target types matches
        if (detectedTargets.at(objectId)->type == objectType) {
          detectedTargets.at(objectId)->isTargetReceivedByWorker = true;
          std::cout << "UAV " << uavId << ": target " << objectId << " received by worker " << workerId << "." << std::endl;
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
  if(nh.hasParam("searcher_node/uavId")){
    nh.getParam("searcher_node/uavId", uavId);
  }
  else {
    std::cout << "Unable to get uavId parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("searcher_node/minTargetRadiusMeters")){
    nh.getParam("searcher_node/minTargetRadiusMeters", minTargetRadiusMeters);
  }
  else {
    std::cout << "UAV " << uavId << ": Unable to get minTargetRadiusMeters parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("searcher_node/missionRadius")){
    nh.getParam("searcher_node/missionRadius", missionRadius);
  }
  else {
    std::cout << "UAV " << uavId << ": Unable to get missionRadius parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("searcher_node/missionStep")){
    nh.getParam("searcher_node/missionStep", missionStep);
  }
  else {
    std::cout << "UAV " << uavId << ": Unable to get missionStep parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("searcher_node/missionAltitude")){
    nh.getParam("searcher_node/missionAltitude", missionAltitude);
  }
  else {
    std::cout << "UAV " << uavId << ": Unable to get missionAltitude parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("searcher_node/serialPort")){
    nh.getParam("searcher_node/serialPort", serialPort);
  }
  else {
    std::cout << "UAV " << uavId << ": Unable to get serialPort parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("searcher_node/baud")){
    nh.getParam("searcher_node/baud", baud);
  }
  else {
    std::cout << "UAV " << uavId << ": Unable to get baud parameter." << std::endl;
    return 0;
  }

  multi_uav_se_mission::CSerial *serial = new multi_uav_se_mission::CSerial();

  std::cout << "UAV " << uavId << ": Connecting on serialPort = " << serialPort << " baud = " << baud << std::endl;

  if(serial->openPort(serialPort, baud)){

    serial->setMessageBlockSize(MESSAGE_BLOCK_SIZE_BYTES);

    multi_uav::Drone *d = new multi_uav::Drone(nh, uavId, false);

    std::thread rosLoopThread(&rosLoop);
    std::thread uavMissionThread(&uavMission, d, missionRadius, missionStep, missionAltitude);
    std::thread uavDetectionThread(&uavDetection, d, minTargetRadiusMeters);
    std::thread communicationSenderThread(&communicationSender, serial, uavId);
    std::thread communicationReceiverThread(&communicationReceiver, serial, uavId);

    rosLoopThread.join();
    uavMissionThread.join();
    uavDetectionThread.join();
    communicationSenderThread.join();
    communicationReceiverThread.join();

  }
  else{
    std::cout << "UAV " << uavId << ": Could not connect to serial port!" << std::endl;
  }

  serial->~CSerial();

  return 0;

}
