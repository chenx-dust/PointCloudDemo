#pragma once

#define LIDAR_TYPE HAP

#if LIDAR_TYPE == HAP
  #define LIDAR_TYPE_STR "hap"
#elif LIDAR_TYPE == MID70
  #define LIDAR_TYPE_STR "mid-70"
#endif

#include <mongocxx/client.hpp>
#include <mongocxx/cursor.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <optional>

namespace db
{
class DbReader
{
  private:
    mongocxx::instance instance;
    mongocxx::client client;
    mongocxx::database db;
    mongocxx::collection collection;
    std::optional<mongocxx::cursor> cursor;
    std::optional<mongocxx::cursor::iterator> iterator;

  public:
    DbReader() = default;
    void connect();
    pcl::PointCloud<pcl::PointXYZ> receive();
    bool available();
};

}
