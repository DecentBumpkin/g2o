#include "json_io.hpp"

void convertFromLidarToWorld(const std::unordered_map<int, std::shared_ptr<Eigen::Vector3d> >& map_src,  
                            std::unordered_map<int, std::shared_ptr<Eigen::Vector3d> >& map_final, 
                            const Eigen::Affine3d& A){
    map_final.clear();
    for(auto& it : map_src){
        map_final.emplace(it.first, std::make_shared<Eigen::Vector3d>( A *(*it.second) ) );
    }
    assert(map_src.size() == map_final.size());
}

int writePointsJSON(const char* name, const std::unordered_map<int, std::shared_ptr<Eigen::Vector3d> >& map){

    rapidjson::Document doc;
    doc.SetObject();
    rapidjson::Value points_js = rapidjson::Value(rapidjson::kArrayType);
    for(auto& it : map)
    {
        rapidjson::Value item(rapidjson::kObjectType);
        rapidjson::Value point_js(rapidjson::kObjectType);
        item.SetInt(it.first);
        point_js.AddMember("id", item, doc.GetAllocator());
        item.SetDouble((*it.second)[0]);
        point_js.AddMember("x", item, doc.GetAllocator());
        item.SetDouble((*it.second)[1]);
        point_js.AddMember("y", item, doc.GetAllocator());
        item.SetDouble((*it.second)[2]);
        point_js.AddMember("z", item, doc.GetAllocator()); /* item is moved */
        points_js.PushBack(point_js, doc.GetAllocator()); /* point is moved */
    }
    doc.AddMember("points", points_js, doc.GetAllocator() ); /* points is "moved" */
    // save to json format
    FILE* fp = fopen(name, "w"); // non-Windows use "w"
    if(!fp) return 1;
    char writeBuffer[65536];
    rapidjson::FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    rapidjson::PrettyWriter<rapidjson::FileWriteStream> fwriter(os);
    fwriter.SetMaxDecimalPlaces(6);
    doc.Accept(fwriter);
    printf("Write to %s\n", name);
    fclose(fp);

    return 0;
}

int readPointsJSON(const char* name, std::unordered_map<int, std::shared_ptr<Eigen::Vector3d> >& map){
    FILE* fp = fopen(name, "r"); // non-Windows use "r"
    if(!fp) return 1;
    printf("Reading %s\n", name);
    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
    rapidjson::Document doc;
    doc.ParseStream(is);

    if (doc.HasMember("points") && doc["points"].IsArray())
    {
        printf("reading lvx %u picked points...\n", doc["points"].Size());
        const rapidjson::Value &array = doc["points"];
        for(auto it = array.Begin(); it != array.End(); it++){
        
            if(it->HasMember("id") && (*it)["id"].IsInt() &&
                it->HasMember("x") && (*it)["x"].IsDouble() &&
                it->HasMember("y") && (*it)["y"].IsDouble() &&
                it->HasMember("z") && (*it)["z"].IsDouble() ){
                int id = (*it)["id"].GetInt();
                double x = (*it)["x"].GetDouble();
                double y = (*it)["y"].GetDouble();
                double z = (*it)["z"].GetDouble();
                printf("Read point %d %lf %lf %lf\n",id, x, y, z);
                map.emplace(id, std::make_shared<Eigen::Vector3d>(x,y,z) );
            }
        }
    }
    fclose(fp);
    return 0;
}