#include "DataBase.h"
#include "DBConfig.h"
#include "DebugConfig.h"
#define DPRINT(x)    DPRINT_DB(x)
#define DPRINTLN(x)  DPRINTLN_DB(x)
#define LOG_FUNC(x) LOG_DB_MSG(x)


DataBase::DataBase() : dbMutex(xSemaphoreCreateMutex()),ready(false){
    LittleFS.mkdir(LOG_DIR);
}

DataBase::~DataBase() {
    if (flushTaskHandle) {
      vTaskDelete(flushTaskHandle);
    }
    if (uploadTaskHandle) {
      vTaskDelete(uploadTaskHandle);
    }
    if (dbMutex != nullptr) {
        vSemaphoreDelete(dbMutex);
    }
    // Free owned LogDirs
    for (auto& kv : logDirs) {
        delete kv.second;
    }
}

bool DataBase::begin() {
  DPRINTLN("Database begin...");
  if (WiFi.status() != WL_CONNECTED) {
    DPRINTLN("Error: WiFi not connected, database can't initialize.");
    ready = false;
    return false;
  }
  if (!connectToDB()) return false;
  DPRINTLN("Wifi CONNECTED");

  ready = true;
  if (!LittleFS.begin()) {
    DPRINTLN("LittleFS mount failed, formatting...");
    LittleFS.format();  // ERASES EVERYTHING in the partition!
    if (LittleFS.begin()) {
      DPRINTLN("LittleFS format + mount OK!");
    } else {
      DPRINTLN("LittleFS format failed — check partition size");
    }
  }  LittleFS.mkdir(LOG_DIR);
  DPRINTLN("Database Directory INITED");
  
  // ✅ Create flush task
  xTaskCreatePinnedToCore(
    flushTaskWrapper,
    "FlushTask",
    4096,
    this,
    2,
    &flushTaskHandle,
    0
  );

  // ✅ Create upload task
  xTaskCreatePinnedToCore(
    uploadTaskWrapper,
    "UploadTask",
    8192,
    this,
    2,
    &uploadTaskHandle,
    0
  );
  DPRINTLN("Database tasks CONNECTED");

  Firebase.RTDB.deleteNode(&fbdo, "/");
  DPRINTLN("Database Cleared");

  return true;
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
  DPRINTLN(Firebase.ready());      
  return Firebase.ready();
}

void DataBase::maybeFlushIfLowMemory() {
  // Check free heap
  // if (ESP.getFreeHeap() < 1024 * 100) {
  //   Serial.println("WARNING: Heap below 40 KB. Forcing log flush!");
  //   uploadTask();
  // }
}

ILogRecord* DataBase::createRecord(const String& category, const String& session, bool block) {
  DPRINTLN("Heap: " + String(ESP.getFreeHeap()));
  ILogRecord* record = nullptr;

  auto it = logDirs.find(category);
  if (it == logDirs.end()) {
        // Take the database mutex in the desired mode
    if (xSemaphoreTake(dbMutex, block ? portMAX_DELAY : 0) != pdTRUE) {
      DPRINTLN("WARNING: Could not create record, dbMutex busy");
      return nullptr;
    }
    // Not found: create new LogDir for this category
    LogDir* newDir = new(std::nothrow) LogDir(category);
    if (!newDir) {
      DPRINTLN("ERROR: Failed to allocate LogDir for category: " + category);
      xSemaphoreGive(dbMutex);
      return nullptr;
    }
    logDirs[category] = newDir;
    xSemaphoreGive(dbMutex);
    record = newDir->createRecord(session, block); // pass 'block' here too
  } else {
    record = it->second->createRecord(session, block); // pass 'block'
  }

  DPRINTLN("Created Record");
  return record; // record may be nullptr if creation failed
}

void DataBase::registerLogDir(LogDir* dir) {
    if (!dir) return;
    xSemaphoreTake(dbMutex, portMAX_DELAY);
    logDirs[dir->getCategory()] = dir;
    xSemaphoreGive(dbMutex);
}

void DataBase::flushTaskWrapper(void* pvParameters) {
  DataBase* db = static_cast<DataBase*>(pvParameters);
  const TickType_t delayTicks = pdMS_TO_TICKS(LOG_FLUSH_INTERVAL_MS);
  while (true) {
    DPRINTLN("----------FLUSHING------------");
    DPRINTLN("on Core " + String(xPortGetCoreID()));
    db->flushAllLogDirs();
    DPRINTLN("------DONE FLUSHING------");
    vTaskDelay(delayTicks);
  }
}

void DataBase::uploadTaskWrapper(void* pvParameters) {
  DataBase* db = static_cast<DataBase*>(pvParameters);
  const TickType_t delayTicks = pdMS_TO_TICKS(LOG_UPLOAD_INTERVAL_MS);
  while (true) {
    DPRINTLN("----------UPLOADING-----------");
    DPRINTLN("on Core " + String(xPortGetCoreID()));
    db->uploadLog();
    DPRINTLN("------DONE UPLOADING------");
    vTaskDelay(delayTicks);
  }
}


void DataBase::flushAllLogDirs() {

  xSemaphoreTake(dbMutex, portMAX_DELAY);
  for (auto& kv : logDirs) {
    kv.second->flushToFile();
  }
  xSemaphoreGive(dbMutex); // Don’t forget to release!
}

void DataBase::uploadLog() {

  FirebaseJson megaWrapper;
  size_t payloadSize = 0; 
  if (!ready || !Firebase.ready()) return;

  String t = LogDir::timeHMSms();
    // Lock access to logDirs map (if you plan to add/remove dirs at runtime)
  xSemaphoreTake(dbMutex, portMAX_DELAY);
  std::vector<std::pair<const String, LogDir*>*> sortedDirs;

  for (auto& kv : logDirs) {
      sortedDirs.push_back(&kv); // store pointer to the map pair
  }

  // Step 2: Sort by LogDir buffered size descending
std::sort(sortedDirs.begin(), sortedDirs.end(),
    [](std::pair<const String, LogDir*>* a,
       std::pair<const String, LogDir*>* b) {
        return a->second->size() > b->second->size();
    });
      for (auto pairPtr : sortedDirs) {
        auto& [name, dir] = *pairPtr;  // dereference, then destructure
        FirebaseJsonArray catData;
        dir->uploadFileToArray(catData, payloadSize, DB_PAYLOAD_SIZE_KB);
        if (catData.size() > 0) {
            megaWrapper.set(name, catData);
        }
    }
  String path = "/LOGS/" + LogDir::timeHMSms();
  DPRINTLN("Data Collecting resulted in ");DPRINTLN(payloadSize);DPRINTLN(" bytes");
  xSemaphoreGive(dbMutex);
  if(payloadSize <= 0)
    return;
  // Send one mega JSON write to root path "/"
  DPRINTLN("Im here!");
  fbdo.clear();
  if (Firebase.RTDB.setJSON(&fbdo, path, &megaWrapper)) {
    DPRINTLN("Mega log uploaded at " + t);
  } else {
    DPRINT("Mega log upload FAILED");
  }
  fbdo.clear();
}
