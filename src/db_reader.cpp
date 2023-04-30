#include "db_reader.hpp"

#include <bsoncxx/builder/basic/kvp.hpp>
#include <cstdint>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/uri.hpp>


namespace db
{

void DbReader::connect()
{
    mongocxx::uri uri("mongodb://localhost:27017");
    client = mongocxx::client(uri);
    db = client["radar"];
    collection = db["lidar_raw_dump"];

    const auto ping_cmd = bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp("ping", 1));
    db.run_command(ping_cmd.view());
    std::cout << "Pinged your deployment. You successfully connected to MongoDB!" << std::endl;

    cursor = collection.find(bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp("device", LIDAR_TYPE_STR)));
    iterator = cursor->begin();
}

pcl::PointCloud<pcl::PointXYZ> DbReader::receive()
{
    pcl::PointCloud<pcl::PointXYZ> points;
    try
    {
        auto raw_points = (**iterator)["points"].get_array().value;
        for (auto &&raw_point : raw_points)
        {
            auto x = raw_point[0].get_int32().value;
            auto y = raw_point[1].get_int32().value;
            auto z = raw_point[2].get_int32().value;
            points.emplace_back(x, y, z);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    ++*iterator;
    return points;
}

bool DbReader::available()
{
    if (cursor)
        return iterator != cursor->end();
    else
        return false;
}
}