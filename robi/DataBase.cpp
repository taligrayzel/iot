#include "DataBase.h"
#include "DBConfig.h"

DataBase::DataBase(){}

bool DataBase::begin(){
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Error: WiFi not connected, database can't initialize.");
      return false;
  }
  return connectToDB();
}
bool DataBase::connectToDB(){
  config.database_url = DB_URL;
  config.signer.tokens.legacy_token = DB_SECRET;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  uint32_t t0 = millis();
  while (!Firebase.ready() && millis() - t0 < 10000) {
    delay(25);
  }
  Serial.println(Firebase.ready());      
  return true;
}

void DataBase::debugPrintToDBString(string x){
  String path = "/DEBUG/" + String(millis());
  Firebase.RTDB.setString(&fbdo, path, (string)x);
}

void DataBase::debugPrintToDBFloat(String what ,float x){
  String path = "/DEBUG/"+what+"/" + String(millis());
  Firebase.RTDB.setFloat(&fbdo, path, (float)x);
}

void DataBase::debugPrintToDB(int x){
  String path = "/DEBUG/" + String(millis());
  Firebase.RTDB.setInt(&fbdo, path, (int)x);
}

static String timeHMSms()
{
  struct timeval tv;
  gettimeofday(&tv, nullptr);          // seconds + microseconds since 1970-01-01
  struct tm tm;
  localtime_r(&tv.tv_sec, &tm);        // convert to Israel time

  char hms[9];                         // "HH-MM-SS"
  strftime(hms, sizeof(hms), "%H-%M-%S", &tm);

  char stamp[16];                      // "HH-MM-SS-123"
  snprintf(stamp, sizeof(stamp), "%s-%03ld", hms, tv.tv_usec / 1000L);

  return String(stamp);
}
