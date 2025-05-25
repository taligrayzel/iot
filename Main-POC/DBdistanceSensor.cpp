#include "DBdistanceSensor.h"
#include "DBConfig.h"

DB::DB(){}

void DB::connectToDB(){
  // WiFi.begin(WIFI_SSID, WIFI_PASS);
  // while (WiFi.status() != WL_CONNECTED) delay(200);
  // Serial.println("connected to WIFI");

  config.database_url = DB_URL;
  config.signer.tokens.legacy_token = DB_SECRET;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void DB::debugPrintToDB(string x){
  String path = "/DEBUG/" + String(millis());
  Firebase.RTDB.setString(&fbdo, path, (string)x);
}

void DB::debugPrintToDB(float x){
  String path = "/DEBUG/" + String(millis());
  Firebase.RTDB.setFloat(&fbdo, path, (float)x);
}

void DB::debugPrintToDB(int x){
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

void DB::uploadDistancesToDB(int left, int front, int right){
  String path = "/sensors/esp32_01/" + timeHMSms();

  FirebaseJson json;
  json.add("left",  left);
  json.add("front", front);
  json.add("right", right);

  if (!Firebase.RTDB.setJSON(&fbdo, path, &json)) 
  {
    Serial.printf("Upload failed: %s\n",
    fbdo.errorReason().c_str());
  }
}

// void DB::uploadIntToDB(int toUpload){
//   String path = "/sensors/esp32_01/distance/" + String(millis());
//   Firebase.RTDB.setInt(&fbdo, path, (int)toUpload);
// }