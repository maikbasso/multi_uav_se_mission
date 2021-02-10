/*
@author Maik Basso <maik@maikbasso.com.br>
*/

#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <multi_uav/Drone.h>
#include <multi_uav_se_mission/TypeParser.h>
#include <multi_uav_se_mission/CSerial.h>
#include <multi_uav_se_mission/Arrays.h>
#include <multi_uav_se_mission/WorkerStatistics.h>

#define MESSAGE_BLOCK_SIZE_BYTES 40

//statistics
int numberOfMessagesSent = 0;
int numberOfMessagesReceived = 0;
int numberOfTasksReceived = 0;
int numberOfAssignTasksForOtherWorkers = 0;
int numberOfCompletedTasks = 0;
ros::Publisher statisticsStrPublisher;
ros::Publisher statisticsMessagesPublisher;

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

  std::string toString(){
    std::stringstream ss;
    ss << "searcherId=" << searcherId;
    ss << ";objectId=" << objectId;
    ss << ";objectType=" << objectType;
    ss << ";lat=" << lat;
    ss << ";lon=" << lon;
    ss << ";alt=" << alt;
    ss << ";yaw=" << yaw;
    ss << ";distance=" << distance;
    ss << ";isForMe=" << isForMe;
    ss << ";isAccomplished=" << isAccomplished;
    return ss.str();
  }

} Task;

std::vector<Task *> tasks;

void print(std::string msgStr){

  // print
  std::cout << msgStr << std::endl;

  // publish message
  std_msgs::String msg;
  msg.data = msgStr;
  statisticsStrPublisher.publish(msg);

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

void printFrame(int uavId, std::string msg, std::vector<unsigned char> frame){

  std::stringstream ss;
  ss << "UAV " << uavId << "= " << msg << " ";
  for (int i = 0; i < frame.size(); i++) {
    int n = (int) multi_uav_se_mission::TypeParser::ucharToInt8t(frame[i]);
    ss << n << " ";
  }
  print(ss.str());

}

void rosLoop(){
  ros::Rate rate(20);
  while (ros::ok()) {
    ros::spin();
    rate.sleep();
  }
}

void publishWorkerStatisticsLoop(){
  ros::NodeHandle nh;
  ros::Publisher statisticsPublisher = nh.advertise<multi_uav_se_mission::WorkerStatistics>("statistics", 1);
  ros::Publisher statisticsTaskArrayPublisher = nh.advertise<std_msgs::String>("statisticsTasks", 1);

  // ros loop
  ros::Rate rate(1);
  while (ros::ok()) {

    // numerical statistics
    multi_uav_se_mission::WorkerStatistics msg;
    msg.numberOfMessagesSent = numberOfMessagesSent;
    msg.numberOfMessagesReceived = numberOfMessagesReceived;
    msg.numberOfTasksReceived = numberOfTasksReceived;
    msg.numberOfAssignTasksForOtherWorkers = numberOfAssignTasksForOtherWorkers;
    msg.numberOfCompletedTasks = numberOfCompletedTasks;
    statisticsPublisher.publish(msg);

    // publish task array
    std::stringstream ssTask;
    for (int i=0; i<tasks.size(); i++) {
      ssTask << i << "=" << tasks[i]->toString() << ";";
    }
    std_msgs::String msgTask;
    msgTask.data = ssTask.str();
    statisticsTaskArrayPublisher.publish(msgTask);

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

  std::stringstream ss;
  ss.precision(20);
  ss << "UAV " << d->parameters.id << "= going to position= {lat= " << gp->getLatitude() << "; lon= " << gp->getLongitude() << "; alt= " << gp->getAltitude() << "; yaw= " << gp->getYaw() << "}";
  print(ss.str());

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
        std::stringstream ss1;
        ss1.precision(20);
        ss1 << "UAV " << d->parameters.id << "= going to target position= {lat= " << gp->getLatitude() << "; lon= " << gp->getLongitude() << "; alt= " << gp->getAltitude() << "; yaw= " << gp->getYaw() << "}";
        print(ss1.str());
        d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

        // do something

        // complete the task
        t->isAccomplished = true;
        numberOfCompletedTasks++;

      }

    }

    // keeps the uav moving in its current position space

    gp->addPositionOffsetInMeters(1*state,1*state);
    state *= -1;

    std::stringstream ss3;
    ss3.precision(20);
    ss3 << "UAV " << d->parameters.id << "= going to position= {lat= " << gp->getLatitude() << "; lon= " << gp->getLongitude() << "; alt= " << gp->getAltitude() << "; yaw= " << gp->getYaw() << "}";
    print(ss3.str());
    d->goToGlobalPosition(gp->getLatitude(), gp->getLongitude(), gp->getAltitude(), gp->getYaw(), true);

  }

  d->~Drone();

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

    numberOfMessagesReceived++;

    // handle all message types
    if((int) messageType == 0){

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

      publishStatisticMessage0(searcherId, objectId, objectType, lat, lon, alt, yaw);

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

        publishStatisticMessage1(searcherId, d->parameters.id, objectId, objectType, distance, 0.0);

        numberOfMessagesSent++;

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

          numberOfTasksReceived++;
        }

      }

    }
    else if((int) messageType == 1){

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

      publishStatisticMessage1(searcherId, workerId, objectId, objectType, distance, 0.0);


      // verificar a lista de tarefas para ver se a tarefa existe
      int taskId = -1;
      for (int i = 0; i < tasks.size(); i++) {
        if(
           tasks.at(i)->searcherId == searcherId &&
           tasks.at(i)->objectId == objectId &&
           tasks.at(i)->objectType == objectType
           ){
          taskId = i;
          break;
        }
      }

      if(taskId != -1){

        // check if task is not for me
        if(tasks.at(taskId)->distance > distance && d->parameters.id != workerId){
          tasks.at(taskId)->isForMe = false;
          numberOfAssignTasksForOtherWorkers++;
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

  statisticsStrPublisher = nh.advertise<std_msgs::String>("statisticsLog", 1);
  statisticsMessagesPublisher = nh.advertise<std_msgs::String>("statisticsMessages", 1);

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
  if(nh.hasParam(ros::this_node::getName() + "/altitude")){
    nh.getParam(ros::this_node::getName() + "/altitude", altitude);
  }
  else {
    ss << "UAV " << uavId << "= Unable to get altitude parameter.";
    print(ss.str());
    return 0;
  }
  if(nh.hasParam(ros::this_node::getName() + "/taskTypes")){

    std::string taskTypesStr;

    nh.getParam(ros::this_node::getName() + "/taskTypes", taskTypesStr);

    std::vector<std::string> taskTypesStrArray = split(taskTypesStr, ',');

    for(int z=0; z < taskTypesStrArray.size(); z++){
      taskTypes.push_back(std::stoi(taskTypesStrArray.at(z)));
    }
  }
  else {
    ss << "UAV " << uavId << "= Unable to get taskTypes parameter.";
    print(ss.str());
    return 0;
  }
  if(nh.hasParam(ros::this_node::getName() + "/taskAssignAtSeconds")){
    nh.getParam(ros::this_node::getName() + "/taskAssignAtSeconds", taskAssignAtSeconds);
  }
  else {
    ss << "UAV " << uavId << "= Unable to get taskAssignAtSeconds parameter.";
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
  print(ss.str());

  if(serial->openPort(serialPort, baud)){

    serial->setMessageBlockSize(MESSAGE_BLOCK_SIZE_BYTES);

    multi_uav::Drone *d = new multi_uav::Drone(nh, uavId, false);

    std::thread rosLoopThread(&rosLoop);
    std::thread publishWorkerStatisticsLoopThread(&publishWorkerStatisticsLoop);
    std::thread communicationThread(&communication, serial, d, taskTypes);
    std::thread taskPerformerThread(&taskPerformer, d, altitude, taskAssignAtSeconds);

    rosLoopThread.join();
    publishWorkerStatisticsLoopThread.join();
    communicationThread.join();
    taskPerformerThread.join();

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
