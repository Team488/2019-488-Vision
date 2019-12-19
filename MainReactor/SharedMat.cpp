#include "opencv2/highgui.hpp"
#include <boost/interprocess/managed_shared_memory.hpp>
#include <iostream>
#include "SharedMat.h"

#ifdef _WIN32
#include <windows.h>
#else  // _WIN32
#include <sys/syscall.h>
#include <linux/futex.h>
static int futex(int *uaddr, int futex_op, int val,
      const struct timespec *timeout, int *uaddr2, int val3)
{
  return syscall(SYS_futex, uaddr, futex_op, val, timeout, uaddr, val3);
}
#endif  // _WIN32

using namespace boost::interprocess;

char *DEMO_MEMORY_NAME = "SharedMemDemo";
char *DEMO_HEADER_NAME = "DemoHeader";

SharedMat::SharedMat(char *name, char *header_name)
    : segment(open_only, name)
{
  HEADER_NAME = header_name;
  std::pair<cv::Mat *, managed_shared_memory::size_type> res;

  // Open Shared Mat Header
  this->header = segment.find<SharedMatHeader>(HEADER_NAME).first;

  this->mat = cv::Mat(
      this->header->size,
      this->header->type,
      segment.get_address_from_handle(this->header->handle));
}

bool SharedMat::waitForFrame()
{
  uint current_frame = header->frame;

#ifdef _WIN32
  while (header->frame == current_frame)
  {
    Sleep(10);
  }
#else  // _WIN32
  // futex (or Fast User Mutex) is a linux kernel feature that lets processes wait for some condition
  // to be true.
  int s = futex(&(header->frame), FUTEX_WAIT, current_frame, NULL, NULL, 0);
  if (s == -1 && errno != EAGAIN)
  {
    std::cerr << "Error waiting for futex: " << errno << std::endl;
    exit(s);
  }
#endif  // _WIN32

  return true;
}

void SharedMatWriter::updateMemory(char *memName, char *headerName, cv::Mat frame)
{
  remover =  ShmRemover(memName);
  segment = managed_shared_memory(create_only, memName, 1 * (frame.total() * frame.elemSize()) + sizeof(SharedMatHeader) + 1024);
  const int data_size = frame.total() * frame.elemSize();
  this->mem_name = memName;

  // Create a region for the header.
  this->header = segment.find_or_construct<SharedMatHeader>(headerName)();

  // Allocate memory for the mat data.
  const SharedMatHeader *shared_mat_data_ptr;
  shared_mat_data_ptr = (SharedMatHeader *)segment.allocate(data_size);

  this->header->size = frame.size();
  this->header->type = frame.type();
  this->header->frame = 0;

  // Set the handle to the allocated region for data.
  this->header->handle = segment.get_handle_from_address(shared_mat_data_ptr);

  this->mat = cv::Mat(
      this->header->size,
      this->header->type,
      segment.get_address_from_handle(this->header->handle));
}

SharedMatWriter::SharedMatWriter(){

};

SharedMatWriter::SharedMatWriter(char *memName, char *headerName, cv::Mat frame)
    : segment(create_only, memName, 1 * (frame.total() * frame.elemSize()) + sizeof(SharedMatHeader) + 1024),
      remover(memName)
{
  const int data_size = frame.total() * frame.elemSize();
  this->mem_name = memName;
  this->header_name = headerName;

  // Create a region for the header.
  this->header = segment.find_or_construct<SharedMatHeader>(headerName)();

  // Allocate memory for the mat data.
  const SharedMatHeader *shared_mat_data_ptr;
  shared_mat_data_ptr = (SharedMatHeader *)segment.allocate(data_size);

  this->header->size = frame.size();
  this->header->type = frame.type();
  this->header->frame = 0;

  // Set the handle to the allocated region for data.
  this->header->handle = segment.get_handle_from_address(shared_mat_data_ptr);

  this->mat = cv::Mat(
      this->header->size,
      this->header->type,
      segment.get_address_from_handle(this->header->handle));
}

bool SharedMatWriter::updateFrame(cv::VideoCapture &capture)
{
  capture >> this->mat;
  header->frame++;
  this->wakeAll();
  return true;
}

bool SharedMatWriter::updateFrame(cv::Mat &inputMat)
{
  inputMat.copyTo(this->mat);
  header->frame++;
  this->wakeAll();
  return true;
}

void SharedMatWriter::wakeAll()
{
#ifdef _WIN32
#else  // _WIN32
  int s = futex(&(header->frame), FUTEX_WAKE, INT_MAX, NULL, NULL, 0);
  if (s == -1 && errno != EAGAIN)
  {
    std::cerr << "Error waking futex: " << errno << std::endl;
    exit(s);
  }
#endif  // _WIN32
}