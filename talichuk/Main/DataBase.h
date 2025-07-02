#ifndef DataBase_H
#define DataBase_H

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "freertos/semphr.h"
#include "LogDir.h"

using namespace std;
#include <map>

class DataBase {
public:
    DataBase();
    ~DataBase();

    bool begin();

    // Create or reuse a LogDir, then create a record in it
    ILogRecord* createRecord(const String& category, const String& session,bool block);

    // Register a new LogDir manually if needed (optional)
    void registerLogDir(LogDir* dir);

    // Periodically called to flush all LogDirs' buffered logs to Firebase
    void uploadTask();
    void flushAllLogDirs();
    void uploadLog(); 

    bool isReady() const { return ready; }

private:
    std::map<String, LogDir*> logDirs;   // Managed log directories by category
    void maybeFlushIfLowMemory();
    SemaphoreHandle_t dbMutex;            // Protects logDirs and general concurrency
    FirebaseData fbdo;
    FirebaseAuth auth;
    FirebaseConfig config;
    bool ready = false;

    TaskHandle_t uploadTaskHandle = nullptr;
    TaskHandle_t flushTaskHandle = nullptr; 

    static void uploadTaskWrapper(void* pvParameters);
    static void flushTaskWrapper(void* pvParameters);

    bool connectToDB();
};

#endif