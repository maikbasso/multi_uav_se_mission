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

void uavMission(multi_uav::Drone *d, std::vector<std::vector<double>> mission, double altitude){

  d->configureToUseGlobalCoordinates();

  d->forceModeOffboard();

  d->arm();

  multi_uav::utils::GlobalPosition *gp = new multi_uav::utils::GlobalPosition(
    d->parameters.position.global.latitude,
    d->parameters.position.global.longitude,
    d->parameters.position.global.altitude,
    d->parameters.orientation.global.yaw
  );

  // adding the takeoff point
  gp->addMetersToAltitude(altitude);

  std::cout.precision(20);
  std::cout << "UAV " << d->parameters.id << ": going to position: {lat: " << gp->getLatitude() << ", lon: " << gp->getLongitude() << ", alt: " << gp->getAltitude() << ", yaw: " << gp->getYaw() << "}" << std::endl;

  d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

  for (int i = 0; i < mission.size() && ros::ok(); i++) {

    std::cout << "UAV " << d->parameters.id << ": going to position " << i << ":{lat: " << gp->getLatitude() << ", lon: " << gp->getLongitude() << ", alt: " << gp->getAltitude() << ", yaw: " << gp->getYaw() << "}" << std::endl;

    gp->setLatitude(mission.at(i).at(0));
    gp->setLongitude(mission.at(i).at(1));

    d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);
  }

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

  ros::Rate rate(1);
  while(ros::ok()){

    cv::Mat image = d->sensors.camera.rgb.image;
    double lat = d->parameters.position.global.latitude;
    double lon = d->parameters.position.global.longitude;
    double alt = d->parameters.position.global.altitude;
    double yaw = d->parameters.orientation.global.yaw;
    bool isTargetSent = false;

    // try to detect a target
    std::vector<int> ids = detect(image);

    for (int i=0; ids.size(); i++) {

      // create a candidate target
      Target *candidateTarget = new Target();

      candidateTarget->image = image;
      candidateTarget->lat = lat;
      candidateTarget->lon = lon;
      candidateTarget->alt = alt;
      candidateTarget->yaw = yaw;
      candidateTarget->isTargetReceivedByWorker = isTargetSent;

      // if the target is valid, then add to a detected targets array
      if(validateTarget(candidateTarget, minTargetRadiusMeters)){
        detectedTargets.push_back(candidateTarget);
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

std::vector<std::string> split(const std::string& s, char delimiter){
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

std::vector<std::vector<double>> loadMission(std::string filePath){

  // open file
  std::ifstream infile(filePath);

  std::vector<std::vector<double>> mission;

  //read file
  std::string line;
  int lineCount = -1;
  while(std::getline(infile, line)){

    lineCount++; // ignoring the first line

    if(lineCount == 0) continue;

    std::vector<std::string> items = split(line, ',');

    if(items.size() == 2){

      std::vector<double> p;

      for (int i=0; i<items.size(); i++) {
        p.push_back(std::stod(items.at(i)));
      }

      mission.push_back(p);

    }
  }

  return mission;

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
      if(searcherId == uavId && ((detectedTargets.size() -1) > objectId)){

        // if target types matches
        if (detectedTargets.at(objectId)->type == objectType) {
          detectedTargets.at(objectId)->isTargetReceivedByWorker = true;
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

  //parameters
  int uavId = 0;
  double altitude = 2.0;
  double minTargetRadiusMeters = 2.0;
  std::string missionPlanCSV = "";
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
  if(nh.hasParam("searcher_node/altitude")){
    nh.getParam("searcher_node/altitude", altitude);
  }
  else {
    std::cout << "UAV " << uavId << ": Unable to get altitude parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("searcher_node/missionPlanCSV")){
    nh.getParam("searcher_node/missionPlanCSV", missionPlanCSV);
  }
  else {
    std::cout << "UAV " << uavId << ": Unable to get missionPlanCSV parameter." << std::endl;
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

  std::vector<std::vector<double>> mission = loadMission(missionPlanCSV);
  multi_uav_se_mission::CSerial *serial = new multi_uav_se_mission::CSerial();

  std::cout << "UAV " << uavId << ": Mission loaded = " << mission.size() << " waypoints." << std::endl;
  std::cout << "UAV " << uavId << ": Connecting on serialPort = " << serialPort << " baud = " << baud << std::endl;

  if(mission.size() > 0 /*&& serial->openPort(serialPort, baud)*/){

    serial->setMessageBlockSize(MESSAGE_BLOCK_SIZE_BYTES);

    multi_uav::Drone *d = new multi_uav::Drone(nh, uavId, false);

    std::thread uavMissionThread(&uavMission, d, mission, altitude);
    //std::thread uavDetectionThread(&uavDetection, d, minTargetRadiusMeters);
    std::thread rosLoopThread(&rosLoop);
    //std::thread communicationSenderThread(&communicationSender, serial, uavId);
    //std::thread communicationReceiverThread(&communicationReceiver, serial, uavId);

    uavMissionThread.join();
    //uavDetectionThread.join();
    rosLoopThread.join();
    //communicationSenderThread.join();
    //communicationReceiverThread.join();

  }
  else{
    std::cout << "UAV " << uavId << ": Could not connect to serial port!" << std::endl;
  }

  return 0;

}
