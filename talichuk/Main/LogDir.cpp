#include "LogDir.h"
#include "DBConfig.h"
#include "DebugConfig.h"
#define DPRINT(x)    DPRINT_DB(x)
#define DPRINTLN(x)  DPRINTLN_DB(x)
#define LOG_FUNC(x) LOG_DB_MSG(x)

LogRecord::LogRecord(LogDir* dir, const String& session) : owner(dir){
  json.set("time", LogDir::timeHMSms());
  json.set("session", session);
}
ILogRecord& LogRecord::set(const String& key, float value){
  json.set(key, value);
  return *this;
}
ILogRecord& LogRecord::set(const String& key, const String& value){
  json.set(key, value);
  return *this;
}
bool LogRecord::commit(bool block){
  return owner->commitRecord(this, block);
}

LogDir::LogDir(const String& category) : category(category)
{
  fileName = String(LOG_DIR) + "/" + this->category + ".log";
  lastCreateTime = millis();
  DPRINTLN("constructing dir: "+ fileName);
  bufferMutex = xSemaphoreCreateMutex();
  fileMutex = xSemaphoreCreateMutex();
}

LogRecord* LogDir::createRecord(const String& session, bool block) {
  // Try to take the mutex:
  unsigned long currTIme = millis();
  if (currTIme - lastCreateTime < minCreateIntervalMs) {
    DPRINTLN("WARNING: rate limited!");
    return nullptr;
  }
  lastCreateTime = currTIme;
  if (xSemaphoreTake(bufferMutex, block ? portMAX_DELAY : 0) != pdTRUE) {
    DPRINTLN("WARNING: Skipped log record, mutex busy");
    return nullptr;
  }

  // Got the mutex:
  if (activeRecords.size() + commitedRecords.size() >= LOG_BUFF_MAX_SIZE) {
    DPRINTLN("WARNING: Exceed buffer limits");
    xSemaphoreGive(bufferMutex); // always release before flush
    if(block)
      flushToFile();
    else
      return nullptr;
    // Retry taking the mutex, same blocking mode:
    if (xSemaphoreTake(bufferMutex, block ? portMAX_DELAY : 0) != pdTRUE) {
      DPRINTLN("WARNING: Skipped log record after flush, mutex busy");
      return nullptr;
    }
  }

  LogRecord* rec = new(std::nothrow) LogRecord(this, session);
  if (!rec) {
    DPRINTLN("ERROR: Failed to allocate LogRecord");
    xSemaphoreGive(bufferMutex);
    return nullptr;
  }

  activeRecords.push_back(rec);
  xSemaphoreGive(bufferMutex);
  return rec;
}

bool LogDir::commitRecord(LogRecord* rec, bool block) {
  // Try to take the mutex in desired mode:
  if (xSemaphoreTake(bufferMutex, block ? portMAX_DELAY : 0) != pdTRUE) {
    DPRINTLN("WARNING: Could not commit record, mutex busy");
    return false;  // skip gracefully if non-blocking and busy
  }

  // Mutex acquired:
  auto it = std::find(activeRecords.begin(), activeRecords.end(), rec);
  if (it != activeRecords.end()) {
    commitedRecords.push_back(*it);
    activeRecords.erase(it);
  }

  xSemaphoreGive(bufferMutex);
  return true;
}

size_t LogDir::size(){
  size_t trueSize = 0;
  if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE) {
    File file = LittleFS.open(fileName, "r");
    trueSize = file ? file.size() : 0;
    if (file) file.close();
    xSemaphoreGive(fileMutex);
  }
  return trueSize;
}


// void LogDir::flushToFirebase(FirebaseData* fbdo){
//   FirebaseJsonArray jsonToUpload;
//   size_t buffSize = 0;
//   if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
//     while (!commitedRecords.empty() && buffSize < 10 * 1024) {
//       LogRecord* rec = commitedRecords.front();
//       commitedRecords.pop_front();
//       String jsonStr = rec->getJson().raw();  // or toString() if needed
//       buffSize += jsonStr.length();
//       jsonToUpload.add(jsonStr);
//       delete rec;
//     }
//     xSemaphoreGive(mutex);
//   }
//   if (jsonToUpload.size() == 0) return;  // nothing to upload
//   FirebaseJson logWrapper;
//   logWrapper.set("log", jsonToUpload);  // upload the copy, not the cleared original
//   String path = "/" + category + "/" + DataBase::timeHMSms();

//   if (Firebase.RTDB.setJSON(fbdo, path, &logWrapper)) { 
//     Serial.println();
//     Serial.println(category + " log uploaded: " + path + " , size: " + String(buffSize));
//     Serial.println();
//   } else {
//     Serial.println("Failed to upload " + category + " log: " + fbdo->errorReason());
//   }
// }

// void LogDir::drainBuffer(FirebaseJsonArray& dst, size_t& currSize, size_t maxKb) {
//   // Lock for thread safety
//   Serial.println("draining...");
//   if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
//     while (!commitedRecords.empty() && currSize < maxKb * 1024) {
//       LogRecord* rec = commitedRecords.front();
//       commitedRecords.pop_front();
//       String jsonStr = rec->getJson().raw();  // or toString() if needed
//       currSize += jsonStr.length();
//       dst.add(jsonStr);
//       Serial.println("...add...");
//       delete rec;
//     }
//     // If retry in progress, drain that first
//     xSemaphoreGive(mutex);
//   }
// }
// void LogDir::_flushToFile_locked() {
//   File file = LittleFS.open(fileName, "a");
//   if (!file) {
//     Serial.println("Failed to open " + fileName);
//     return;
//   }

//   size_t fileSize = 0;

//   while (!commitedRecords.empty()) {
//     LogRecord* rec = commitedRecords.front();
//     commitedRecords.pop_front();

//     String jsonStr = rec->getJson().raw();
//     file.println(jsonStr); // one JSON per line
//     fileSize += jsonStr.length();

//     delete rec;
//   }

//   file.close();
//   Serial.println("Flushed " + String(fileSize) + " bytes to file " + fileName);
// }


void LogDir::flushToFile() {
  // 1️⃣ Lock buffer and collect committed records
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  std::deque<LogRecord*> toWrite = std::move(commitedRecords);
  commitedRecords.clear();
  xSemaphoreGive(bufferMutex);

  if (toWrite.empty()) return;

  // 2️⃣ Lock file access
  xSemaphoreTake(fileMutex, portMAX_DELAY);

  // Write to a temporary file first
  String tmpFile = fileName + ".tmp";
  File tmp = LittleFS.open(tmpFile, "a");
  if (!tmp) {
    DPRINTLN("ERROR: Failed to open temp file for flushing!");
    for (auto rec : toWrite) delete rec;
    xSemaphoreGive(fileMutex);
    return;
  }

  for (auto rec : toWrite) {
    String json = rec->getJson().raw();
    if (!tmp.println(json)) {
      DPRINTLN("ERROR: Failed to write a record");
    }
    delete rec;
  }

  tmp.flush();
  tmp.close();

  // Replace original log file atomically
  if (LittleFS.exists(fileName)) {
    LittleFS.remove(fileName);
  }

  if (!LittleFS.rename(tmpFile, fileName)) {
    DPRINTLN("ERROR: Failed to rename temp file to original!");
  } else {
    DPRINTLN("Flush to file successful");
  }

  // 3️⃣ Check size and trim if necessary
  File checkFile = LittleFS.open(fileName, "r");
  if (checkFile) {
    size_t size = checkFile.size();
    checkFile.close();
    if (size > LOG_FILE_MAX_SIZE) {
      DPRINTLN("File exceeds max size (" + String(size) + "), trimming...");
      trimDoc();  // Will acquire fileMutex internally
    }
  }

  // 4️⃣ Release file lock
  xSemaphoreGive(fileMutex);
}


// void LogDir::flushToFile() {
  
//   if (xSemaphoreTake(bufferMutex, portMAX_DELAY) == pdTRUE) {
//     _flushToFile_locked();
//     xSemaphoreGive(fileMutex);
//   } else {
//     Serial.println("Failed to take mutex in flushToFile()");
//   }
// }

void LogDir::uploadFileToFirebase(FirebaseData* fbdo) {
  xSemaphoreTake(fileMutex, portMAX_DELAY);

  File file = LittleFS.open(fileName, "r");
  if (!file) {
    DPRINTLN("No file to upload for " + category);
    xSemaphoreGive(fileMutex);
    return;
  }

  FirebaseJsonArray uploadArray;
  size_t payloadSize = 0;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    payloadSize += line.length();
    uploadArray.add(line);
    if (payloadSize > 3 * 1024) break;
  }
  file.close();

  if (uploadArray.size() == 0) {
    xSemaphoreGive(fileMutex);
    return;
  }

  FirebaseJson wrapper;
  wrapper.set("log", uploadArray);
  String path = "/" + category + "/" + LogDir::timeHMSms();

  if (Firebase.RTDB.setJSON(fbdo, path, &wrapper)) {
    DPRINTLN("Uploaded " + category + " log chunk to " + path);
    LittleFS.remove(fileName);
  } else {
    DPRINTLN("Failed to upload " + category + " log chunk");
  }

  xSemaphoreGive(fileMutex);
}

void LogDir::uploadFileToArray(FirebaseJsonArray& dst, size_t& currSize, size_t maxKb) {
  xSemaphoreTake(fileMutex, portMAX_DELAY);

  String originalFile = this->fileName;
  String tempFile = originalFile + ".tmp";

  DPRINTLN("Searching for " + originalFile);
  File file = LittleFS.open(originalFile, "r");
  if (!file) {
    DPRINTLN("No file to upload for " + category);
    xSemaphoreGive(fileMutex);
    return;
  }
  DPRINTLN("Searching Success");

  std::vector<String> uploadedLines;
  std::vector<String> leftoverLines;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();  // remove whitespace including \r
    if (line.isEmpty()) {
      DPRINTLN("Empty or whitespace-only line. Skipping.");
      continue;
    }

    if (line.length() > maxKb * 1024) {
      DPRINTLN("Line too long. Dropping.");
      continue;
    }

    if (currSize + line.length() <= maxKb * 1024) {
      currSize += line.length();
      uploadedLines.push_back(line);
    } else {
      leftoverLines.push_back(line);
      break;  // don't exceed limit
    }
  }
  file.close();

  // Add lines to Firebase array
  for (const auto& l : uploadedLines) {
    dst.add(l);
  }
  DPRINTLN("upload success");

  // Write leftover lines to a temp file
  File tmp = LittleFS.open(tempFile, "w");
  if (!tmp) {
    DPRINTLN("Failed to open temp file for leftovers!");
    xSemaphoreGive(fileMutex);
    return;
  }

  for (const auto& l : leftoverLines) {
    if (!tmp.println(l)) {
      DPRINTLN("Failed to write leftover line.");
    }
  }

  tmp.flush();
  tmp.close();
  DPRINTLN("successful upload leftovers");

  // Replace original file with leftovers
  if (LittleFS.exists(originalFile)) {
    LittleFS.remove(originalFile);
  }
  if (!LittleFS.rename(tempFile, originalFile)) {
    DPRINTLN("Failed to rename temp file for " + category);
  } else {
    DPRINTLN("successful swap");
  }

  DPRINTLN("Prepared " + String(uploadedLines.size()) +
                 " lines to upload. Leftover: " + String(leftoverLines.size()));

  xSemaphoreGive(fileMutex);
}



LogDir::~LogDir() {
  if (bufferMutex != nullptr) {
    vSemaphoreDelete(bufferMutex);
  }
  if (fileMutex != nullptr) {
    vSemaphoreDelete(fileMutex);
  }
  for (auto recPtr : activeRecords) {
    delete recPtr;
  }
  for (auto recPtr : commitedRecords) {
    delete recPtr;
  }
  activeRecords.clear();
  commitedRecords.clear();
}

void LogDir::trimDoc() {
  xSemaphoreTake(fileMutex, portMAX_DELAY);

  File file = LittleFS.open(fileName, "r");
  if (!file) {
    DPRINTLN("No file to trim.");
    xSemaphoreGive(fileMutex);
    return;
  }

  std::vector<String> lines;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (!line.isEmpty()) {
      lines.push_back(line);
    }
  }
  file.close();

  if (lines.size() <= 1) {
    DPRINTLN("Too few lines to trim.");
    xSemaphoreGive(fileMutex);
    return;
  }

  size_t numToKeep = lines.size() / 2;
  std::vector<String> trimmed(lines.end() - numToKeep, lines.end());

  File out = LittleFS.open(fileName + ".tmp", "w");
  if (!out) {
    DPRINTLN("Failed to open temp file for trimming.");
    xSemaphoreGive(fileMutex);
    return;
  }

  for (const auto& line : trimmed) {
    out.println(line);
  }

  out.flush();
  out.close();

  LittleFS.remove(fileName);
  if (!LittleFS.rename(fileName + ".tmp", fileName)) {
    DPRINTLN("Failed to rename trimmed file.");
  } else {
    DPRINTLN("Successfully trimmed file to " + String(numToKeep) + " lines.");
  }

  xSemaphoreGive(fileMutex);
}

String LogDir::timeHMSms()
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