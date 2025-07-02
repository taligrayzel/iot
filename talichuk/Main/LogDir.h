#ifndef LOGDIR_H
#define LOGDIR_H

#include <Firebase_ESP_Client.h>
#include "freertos/semphr.h"
#include <deque>
#include "DBConfig.h"

class ILogRecord {
public:
    virtual ILogRecord& set(const String& key, float value) = 0;
    virtual ILogRecord& set(const String& key, const String& value) = 0;
    virtual bool commit(bool block) = 0;  // tells LogDir: "I’m ready to be uploaded"
    virtual ~ILogRecord() {}
};

class LogDir;

class LogRecord : public ILogRecord {
private:
    FirebaseJson json;
    LogDir* owner;
public:
    LogRecord(LogDir* dir, const String& session);
    ILogRecord& set(const String& key, float value) override;
    ILogRecord& set(const String& key, const String& value) override;
    bool commit(bool block) override;

    FirebaseJson& getJson() { return json; }
};

class LogDir {
private:
    String category;
    String fileName;
    std::deque<LogRecord*> activeRecords;  // store pointers
    std::deque<LogRecord*> commitedRecords;  // store pointers
    SemaphoreHandle_t fileMutex;
    SemaphoreHandle_t bufferMutex;
    unsigned long lastCreateTime = 0;
    const size_t minCreateIntervalMs = (size_t)1000.0/LOG_RATE_LIMIT;  // Example: allow max 10 records/sec
public:
    LogDir(const String& category);
    ~LogDir();

    LogRecord* createRecord(const String& session, bool block);
    bool commitRecord(LogRecord* rec, bool block);
   // void flushToFirebase(FirebaseData* fbdo);
   // void drainBuffer(FirebaseJsonArray& dst, size_t& currSize, size_t maxKb);
    void uploadFileToArray(FirebaseJsonArray& dst, size_t& currSize, size_t maxKb);
    void uploadFileToFirebase(FirebaseData* fbdo);
    
    void flushToFile();
    void trimDoc();
    size_t approxBufferSize;  // estimated size of buffered logs in bytes
    size_t size();
    String getCategory() const { return category; }
    String getBackUpFileName() const { return fileName; }
    static String timeHMSms();
};


#endif
