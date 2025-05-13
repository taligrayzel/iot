#ifndef DBDISTANCESENSOR_H
#define DBDISTANCESENSOR_H

#include <WiFi.h>
#include <time.h>


#define FIREBASE_DISABLE_FCM
#define FIREBASE_DISABLE_STORAGE
#define FIREBASE_DISABLE_GCLOUD_FUNCTIONS
#include <Firebase_ESP_Client.h>


class DB {
public:
  DB();
  void connectToDB();
  //void uploadsDistanceSensoresToDB(int toUpload);
  void setTime();
  void uploadDistancesToDB(int left, int front, int right);


private:
  FirebaseData  fbdo;
  FirebaseAuth  auth;           // not used with legacy secret
  FirebaseConfig config;

};


#endif