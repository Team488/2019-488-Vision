/**
 * This file contains utilities that make it easy to share image data using a
 * shared memory segment.
 * 
 * TODO: Protect readers from reading partial frames.
 * Probably what we want is for readers to copy a frame (while protected by a
 * shared lock) before doing any processing on it. The boost interprocess
 * mutex will deadlock if a process holding one crashes, so we'd probably
 * want to use a linux-specific robust lock.
 * 
 */
#ifndef SHARED_MAT_H 
#define SHARED_MAT_H 1

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <opencv2/opencv.hpp>

using namespace boost::interprocess;

typedef struct {
  cv::Size  size;
  int       type;
  int       frame;
  managed_shared_memory::handle_t handle;
} SharedMatHeader;

extern char *DEMO_MEMORY_NAME;
extern char *DEMO_HEADER_NAME;

/**
 * RAII helper for working with shared memory. Deletes the shared memory
 * when created and when destroyed. An object that contains this object is
 * guaranteed to have exclusive control of the named shared memory object.
 * 
 * Not intended for public use.
 */
class ShmRemover {
  public:
    ShmRemover() {};
    ShmRemover(char* name) : name(name) { shared_memory_object::remove(name); }
    ~ShmRemover() { shared_memory_object::remove(name); }
    void setName(char* name) { shared_memory_object::remove(name); }

  private:
  char* name;
};

/**
 * Reads a matrix out of a named shared memory segment. 
 */
class SharedMat {
  public:
    SharedMat(char* memoryName, char* headerName);

    // Waits indefinitely for a new frame to be ready.
    bool waitForFrame();
    cv::Mat mat;

    // No implict copy constructor
    SharedMat(const SharedMat& rhs) = delete;
    SharedMat& operator=(const SharedMat& rhs) = delete;
  private:
    char * HEADER_NAME;
    SharedMatHeader *header;
    managed_shared_memory segment;
};

/**
 * Helper class for writing frames to a named shared memory segment.
 */
class SharedMatWriter {
  public:
    SharedMatWriter(char* memoryName, char* headerName, cv::Mat frame);

    SharedMatWriter();
    
    void updateMemory(char* memoryName, char* headerName, cv::Mat frame);

    /**
     * Updates the shared frame. This increments the frame counts
     * and notifies any consumers that the new frame is ready.
     */
    bool updateFrame(cv::VideoCapture &capture);

    /**
     * Updates the shared frame as above, just with a direct Mat clone
     */
    bool updateFrame(cv::Mat &inputMat);

    cv::Mat mat;

    // No implict copy constructor
    SharedMatWriter(const SharedMatWriter& rhs) = delete;
    SharedMatWriter& operator=(const SharedMatWriter& rhs) = delete;

  private:
    ShmRemover remover;
    SharedMatHeader *header;
    managed_shared_memory segment;
    char* mem_name;
    char* header_name;

    void wakeAll();
};


#endif // #ifndef SHARED_MAT_H