/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <string>
#include <vector>
#include <map>
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
} Task;

typedef struct WorkerCandidate {
  int workerId;
  double distance;
  std::chrono::seconds time;
} WorkerCandidate;

std::vector<Task *> tasks;
std::map<int, WorkerCandidate *> workerCandidates;

void rosLoop(){
  ros::Rate rate(20);
  while (ros::ok()) {
    ros::spin();
    rate.sleep();
  }
}

void taskPerformer(multi_uav::Drone *d, double altitude){

  // go to the target and do something

  d->configureToUseGlobalCoordinates();

  d->forceModeOffboard();

  d->arm();

  d->takeOff(altitude);

  ros::Rate rate(1);
  while(ros::ok()){

    for (auto &t : tasks) {

      if(!t->isAccomplished && t->isForMe){

        // go to the task
        d->goToGlobalPosition(t->lat, t->lon, altitude, t->yaw, true);

        // do something

        // complete the task
        t->isAccomplished = true;

      }

    }

    rate.sleep();
  }

  d->land();

  d->~Drone();

}

void communication(multi_uav_se_mission::CSerial *serial, multi_uav::Drone *d, std::vector<int> taskTypes){

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

    // handle all message types
    if(messageType != (int8_t) 0){

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

        Task *t = new Task();

        t->searcherId = searcherId;
        t->objectId = objectId;
        t->objectType = objectType;
        t->lat = lat;
        t->lon = lon;
        t->alt = alt;
        t->yaw = yaw;
        t->distance = distance;
        t->isForMe = false;
        t->isAccomplished = false;

        tasks.push_back(t);

      }

    }
    else if(messageType != (int8_t) 1){

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

      // adiciono em uma lista de candidatos para a tarefa
      WorkerCandidate *wc = new WorkerCandidate();
      wc->workerId = workerId;
      wc->distance = distance;
      wc->time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());

      workerCandidates[taskId] = wc;


    }

  }

  serial->~CSerial();
}

// passado um tempo a ser definido, se possuir a menor distancia a tarefa em questão é pra mim
void workerAssign(int workerAssignAtSeconds){

  ros::Rate rate(1);
  while (ros::ok()) {

    for (int i = 0; tasks.size(); i++) {

      if(
         tasks.at(i)->isAccomplished == false &&
         tasks.at(i)->isForMe == false
         ){

        auto minTime = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch());

        for (auto & wc : workerCandidates) {

          if(wc.first == i){

            // verify if the distance
            if(wc.second->distance < tasks.at(i)->distance){
              tasks.at(i)->isAccomplished = true;
            }

            if(minTime > wc.second->time){
              minTime = wc.second->time;
            }

          }

        }

        // if time is past than limitTime work is for me
        std::chrono::seconds sec(workerAssignAtSeconds);
        if(minTime >= sec){
          tasks.at(i)->isForMe = true;
        }

      }

    }

    rate.sleep();
  }

}

int main(int argc, char **argv){
  ros::init(argc, argv, "worker_node");
  ros::NodeHandle nh;

  // parameters
  int uavId = 0;
  double altitude = 2.0;
  std::vector<int> taskTypes;
  int workerAssignAtSeconds = 5;
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
  if(nh.hasParam("searcher_node/altitude")){
    nh.getParam("searcher_node/altitude", altitude);
  }
  else {
    std::cout << "Unable to get altitude parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("searcher_node/taskTypes")){
    nh.getParam("searcher_node/taskTypes", taskTypes);
  }
  else {
    std::cout << "Unable to get taskTypes parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("searcher_node/workerAssignAtSeconds")){
    nh.getParam("searcher_node/workerAssignAtSeconds", workerAssignAtSeconds);
  }
  else {
    std::cout << "Unable to get workerAssignAtSeconds parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("searcher_node/serialPort")){
    nh.getParam("searcher_node/serialPort", serialPort);
  }
  else {
    std::cout << "Unable to get serialPort parameter." << std::endl;
    return 0;
  }
  if(nh.hasParam("searcher_node/baud")){
    nh.getParam("searcher_node/baud", baud);
  }
  else {
    std::cout << "Unable to get baud parameter." << std::endl;
    return 0;
  }

  multi_uav_se_mission::CSerial *serial = new multi_uav_se_mission::CSerial();

  std::cout << "Connecting on serialPort = " << serialPort << " baud = " << baud << std::endl;

  if(serial->openPort(serialPort, baud)){

    serial->setMessageBlockSize(MESSAGE_BLOCK_SIZE_BYTES);

    multi_uav::Drone *d = new multi_uav::Drone(nh, uavId, false);

    std::thread rosLoopThread(&rosLoop);
    std::thread communicationThread(&communication, serial, d, taskTypes);
    std::thread taskPerformerThread(&taskPerformer, d, altitude);
    std::thread workerAssignThread(&workerAssign, workerAssignAtSeconds);

    rosLoopThread.join();
    communicationThread.join();
    taskPerformerThread.join();
    workerAssignThread.join();

  }
  else{
    std::cout << "Could not connect to serial port!" << std::endl;
  }

  return 0;

}
