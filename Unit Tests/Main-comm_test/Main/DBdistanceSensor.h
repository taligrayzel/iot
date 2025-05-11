#ifndef DBDISTANCESENSOR_H
#define DBDISTANCESENSOR_H

#include <WiFi.h>
#include <Firebase_ESP_Client.h>


class DB {
public:
  DB();
  void connectToDB();
  void uploadIntToDB(int toUpload);


private:
  FirebaseData  fbdo;
  FirebaseAuth  auth;           // not used with legacy secret
  FirebaseConfig config;

};


#endif