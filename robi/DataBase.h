#ifndef DataBase_H
#define DataBase_H

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <string>
#include <type_traits>
using namespace std;


class DataBase {
  bool connectToDB();
public:
  DataBase();
  bool begin();
  void debugPrintToDBFloat(String what ,float x);
  void debugPrintToDBString(string x);

  void debugPrintToDB(int x);

private:
  FirebaseData  fbdo;
  FirebaseAuth  auth;           // not used with legacy secret
  FirebaseConfig config;

};

#endif