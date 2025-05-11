#include "DBdistanceSensor.h"
#include "DBConfig.h"

DB::DB(){}

void DB::connectToDB(){
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(200);
  Serial.println("connected to WIFI");

  config.database_url = DB_URL;
  config.signer.tokens.legacy_token = DB_SECRET;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void DB::uploadIntToDB(int toUpload){
  String path = "/sensors/esp32_01/distance/" + String(millis());
  Firebase.RTDB.setInt(&fbdo, path, (int)toUpload);
}