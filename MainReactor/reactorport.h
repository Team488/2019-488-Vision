#ifndef REACTORPORT_H
#define REACTORPORT_H 1

#include <opencv2/opencv.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

/*
 * from http://opencvkazukima.codeplex.com/SourceControl/changeset/view/19055#interprocess/src/client.cpp
 *
 */
typedef struct{
  cv::Size  size;
  int       type;
  int       buffPosition;
  boost::interprocess::managed_shared_memory::handle_t handle;
} SharedImageHeader;

const char *MEMORY_NAME = "ReactorFlow";

cv::Mat get_shared_mat(const char *name);
cv::Mat create_shared_mat(const char *name, cv::Mat frame);

#endif // #ifndef INTERPROCESS_COMMON_H
