#ifndef DBDISTANCESENSOR_H
#define DBDISTANCESENSOR_H

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <string>
using namespace std;


class DB {
public:
  DB();
  void connectToDB();
//  void uploadIntToDB(int toUpload);
  void debugPrintToDB(string x);
  void debugPrintToDB(int x);
  void debugPrintToDB(float x);
  void uploadDistancesToDB(int left, int front, int right);


private:
  FirebaseData  fbdo;
  FirebaseAuth  auth;           // not used with legacy secret
  FirebaseConfig config;

};


#endif