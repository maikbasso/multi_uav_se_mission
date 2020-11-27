/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <string>
#include <vector>
#include <chrono>
#include <ros/ros.h>
#include <multi_uav/Drone.h>
#include <multi_uav_se_mission/TypeParser.h>
#include <multi_uav_se_mission/CSerial.h>
#include <multi_uav_se_mission/Arrays.h>

#define MESSAGE_BLOCK_SIZE_BYTES 40

typedef struct Task {
  int searcherId;
  int objectId;
  int objectType;
  double lat;
  double lon;
  double alt;
  double yaw;
  double distance;
  bool isForMe;
  bool isAccomplished;
  std::chrono::seconds createdAt;
} Task;

typedef struct WorkerCandidate {
  int taskId;
  int workerId;
  double distance;
} WorkerCandidate;

std::vector<Task *> tasks;
std::vector<WorkerCandidate *> workerCandidates;

void rosLoop(){
  ros::Rate rate(20);
  while (ros::ok()) {
    ros::spin();
    rate.sleep();
  }
}

void taskPerformer(multi_uav::Drone *d, double altitude, int taskAssignAtSeconds){

  // go to the target and do something

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

  std::cout << "UAV " << d->parameters.id << ": going to position: {lat: " << gp->getLatitude() << ", lon: " << gp->getLongitude() << ", alt: " << gp->getAltitude() << ", yaw: " << gp->getYaw() << "}" << std::endl;

  d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

  int state = 1;

  while(ros::ok()){

    for (auto &t : tasks) {

      auto currentTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());
      std::chrono::seconds sec(taskAssignAtSeconds);

      if(
         t->isAccomplished == false &&
         t->isForMe == true &&
         ((currentTime - t->createdAt) > sec)
         ){

        // go to the task
        gp->setLatitude(t->lat);
        gp->setLongitude(t->lon);
        gp->setYaw(t->yaw);
        std::cout << "UAV " << d->parameters.id << ": going to target position: {lat: " << gp->getLatitude() << ", lon: " << gp->getLongitude() << ", alt: " << gp->getAltitude() << ", yaw: " << gp->getYaw() << "}" << std::endl;
        d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

        // do something

        // complete the task
        t->isAccomplished = true;

      }

    }

    // keeps the uav moving in its current position space

    gp->addPositionOffsetInMeters(1*state,1*state);
    state *= -1;

    std::cout << "UAV " << d->parameters.id << ": going to position: {lat: " << gp->getLatitude() << ", lon: " << gp->getLongitude() << ", alt: " << gp->getAltitude() << ", yaw: " << gp->getYaw() << "}" << std::endl;
    d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

  }

  d->~Drone();

}

void printFrame(int uavId, std::string msg, std::vector<unsigned char> frame){

  std::cout << "UAV " << uavId << ": " << msg << " ";
  for (int i = 0; i < frame.size(); i++) {
    int n = (int) multi_uav_se_mission::TypeParser::ucharToInt8t(frame[i]);
    std::cout << n << " ";
  }
  std::cout << std::endl;

}

void communication(multi_uav_se_mission::CSerial *serial, multi_uav::Drone *d, std::vector<int> taskTypes){

  while (ros::ok() && serial->isOpened()) {

    // received frame
    ////////////////////////////
    std::vector<unsigned char> frame = serial->readDataBlock();

    printFrame(d->parameters.id, "received message", frame);

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

    // handle all message types
    if(messageType == (int8_t) 0){

      //searcher id
      int searcherId = multi_uav_se_mission::TypeParser::ucharArrayToInt(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_INT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_INT_BYTES;

      //object id
      int objectId = multi_uav_se_mission::TypeParser::ucharArrayToInt(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_INT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_INT_BYTES;

      //object type
      int objectType = multi_uav_se_mission::TypeParser::ucharArrayToInt(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_INT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_INT_BYTES;

      //position
      float lat = multi_uav_se_mission::TypeParser::ucharArrayToFloat(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_FLOAT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_FLOAT_BYTES;
      float lon = multi_uav_se_mission::TypeParser::ucharArrayToFloat(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_FLOAT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_FLOAT_BYTES;
      float alt = multi_uav_se_mission::TypeParser::ucharArrayToFloat(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_FLOAT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_FLOAT_BYTES;
      float yaw = multi_uav_se_mission::TypeParser::ucharArrayToFloat(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_FLOAT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_FLOAT_BYTES;


      // verify if can perform the task
      if(multi_uav_se_mission::Arrays::contains(taskTypes, objectType)){

        double distance = sqrt(pow((lat)-(d->parameters.position.global.latitude), 2)+pow((lon)-(d->parameters.position.global.longitude),2));

        std::vector<unsigned char> msg1Frame;

        // start frame
        int8_t frameIdentification = -128;
        msg1Frame.push_back(multi_uav_se_mission::TypeParser::int8tToUchar(frameIdentification));

        // message type
        int8_t messageType = 1;
        msg1Frame.push_back(multi_uav_se_mission::TypeParser::int8tToUchar(messageType));

        // searcher id
        multi_uav_se_mission::Arrays::emplaceBack(msg1Frame, multi_uav_se_mission::TypeParser::intToUcharArray(searcherId));

        // worker id
        multi_uav_se_mission::Arrays::emplaceBack(msg1Frame, multi_uav_se_mission::TypeParser::intToUcharArray(d->parameters.id));

        // object id
        multi_uav_se_mission::Arrays::emplaceBack(msg1Frame, multi_uav_se_mission::TypeParser::intToUcharArray(objectId));

        // object type
        multi_uav_se_mission::Arrays::emplaceBack(msg1Frame, multi_uav_se_mission::TypeParser::intToUcharArray(objectType));

        // distance
        multi_uav_se_mission::Arrays::emplaceBack(msg1Frame, multi_uav_se_mission::TypeParser::floatToUcharArray(distance));

        serial->writeDataInBlocks(msg1Frame);

        printFrame(d->parameters.id, "sending message", msg1Frame);

        // verify if task does not exists
        bool canCreateNewTask = true;
        for (auto &task : tasks) {

          if(
             task->searcherId == searcherId &&
             task->objectId == objectId &&
             task->objectType == objectType
             ){
            canCreateNewTask = false;
            break;
          }

        }

        if(canCreateNewTask){

          Task *t = new Task();

          t->searcherId = searcherId;
          t->objectId = objectId;
          t->objectType = objectType;
          t->lat = lat;
          t->lon = lon;
          t->alt = alt;
          t->yaw = yaw;
          t->distance = distance;
          t->isForMe = true;
          t->isAccomplished = false;
          t->createdAt = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());

          tasks.push_back(t);
        }

      }

    }
    else if(messageType == (int8_t) 1){

      //searcher id
      int searcherId = multi_uav_se_mission::TypeParser::ucharArrayToInt(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_INT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_INT_BYTES;

      //worker id
      int workerId = multi_uav_se_mission::TypeParser::ucharArrayToInt(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_INT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_INT_BYTES;

      //object id
      int objectId = multi_uav_se_mission::TypeParser::ucharArrayToInt(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_INT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_INT_BYTES;

      //object type
      int objectType = multi_uav_se_mission::TypeParser::ucharArrayToInt(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_INT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_INT_BYTES;

      // distance
      float distance = multi_uav_se_mission::TypeParser::ucharArrayToFloat(multi_uav_se_mission::Arrays::subvector(frame, i, multi_uav_se_mission::TypeParser::SIZE_FLOAT_BYTES));
      i += multi_uav_se_mission::TypeParser::SIZE_FLOAT_BYTES;


      // verificar a lista de tarefas para ver se a tarefa existe
      int taskId = -1;
      for (int i = 0; i < tasks.size(); i++) {
        if(
           tasks.at(i)->searcherId == searcherId &&
           tasks.at(i)->objectId == objectId &&
           tasks.at(i)->objectType == objectType
           ){
          taskId = i;
        }
      }

      if(taskId != -1){

        // worker candidate
        WorkerCandidate *wc = new WorkerCandidate();
        wc->taskId = taskId;
        wc->workerId = workerId;
        wc->distance = distance;

        // check if task is not for me
        if(tasks.at(taskId)->distance > wc->distance){
          tasks.at(taskId)->isForMe = false;
        }

        // create the new worker candidate
        bool canAddWorkerCandidate = true;

        for (int i = 0; i < workerCandidates.size(); i++) {
          if(
             workerCandidates.at(i)->taskId == taskId &&
             workerCandidates.at(i)->workerId == workerId
             ){
            canAddWorkerCandidate = false;
            break;
          }
        }

        if(canAddWorkerCandidate){
          workerCandidates.push_back(wc);
        }

      }

    }

  }

  serial->~CSerial();
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

int main(int argc, char **argv){
  ros::init(argc, argv, "worker_node");
  ros::NodeHandle nh;

  // configuring cout precision
  std::cout.precision(20);

  // parameters
  int uavId = 0;
  double altitude = 2.0;
  std::vector<int> taskTypes;
  int taskAssignAtSeconds = 5;
  std::string serialPort = "";
  int baud = 9600;

  //get parameters
  if(nh.hasParam("worker_node/uavId")){
    nh.getParam("worker_node/uavId", uavId);
  }
  else {
    std::cout << "Unable to get uavId parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("worker_node/altitude")){
    nh.getParam("worker_node/altitude", altitude);
  }
  else {
    std::cout << "UAV " << uavId << ": Unable to get altitude parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("worker_node/taskTypes")){

    std::string taskTypesStr;

    nh.getParam("worker_node/taskTypes", taskTypesStr);

    std::vector<std::string> taskTypesStrArray = split(taskTypesStr, ',');

    for(int z=0; z < taskTypesStrArray.size(); z++){
      taskTypes.push_back(std::stoi(taskTypesStrArray.at(z)));
    }
  }
  else {
    std::cout << "UAV " << uavId << ": Unable to get taskTypes parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("worker_node/taskAssignAtSeconds")){
    nh.getParam("worker_node/taskAssignAtSeconds", taskAssignAtSeconds);
  }
  else {
    std::cout << "UAV " << uavId << ": Unable to get taskAssignAtSeconds parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("worker_node/serialPort")){
    nh.getParam("worker_node/serialPort", serialPort);
  }
  else {
    std::cout << "UAV " << uavId << ": Unable to get serialPort parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("worker_node/baud")){
    nh.getParam("worker_node/baud", baud);
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
    std::thread communicationThread(&communication, serial, d, taskTypes);
    std::thread taskPerformerThread(&taskPerformer, d, altitude, taskAssignAtSeconds);

    rosLoopThread.join();
    communicationThread.join();
    taskPerformerThread.join();

  }
  else{
    std::cout << "UAV " << uavId << ": Could not connect to serial port!" << std::endl;
  }

  serial->~CSerial();

  return 0;

}
