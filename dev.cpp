#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <assert.h>

int main()
{
    YAML::Node node = YAML::LoadFile("./test.yaml");
    //获取类型
    std::cout << node.Type() << std::endl; //4
    std::cout << node["name"].Type() << std::endl;//2

    //获取内容
    std::cout << node["name"].as<std::string>() << std::endl;//xiaoming
    std::cout << node["sex"].as<std::string>() << std::endl;//male
    std::cout << node["age"].as<int>() << std::endl;//18
    std::cout << node["system"]["port"].as<std::string>() << std::endl;//8080
    std::cout << node["system"]["value"].as<std::string>() << std::endl;//0
    for(auto it = node["system"]["int_vec"].begin(); it != node["system"]["int_vec"].end(); it++)
        std::cout << *it << std::endl;//10 20

    //方式2
    for(auto it = node.begin(); it != node.end(); it++)//first指向key , second指向value
        std::cout << it->first << "  " << it->second << std::endl;
    /*读取结果
    name  xiaoming
    sex  male
    age  18
    system  port: 0
    value: 0
    int_vec: [10, 20]
    */

    //读取不存在的node值，报YAML::TypedBadConversion异常
    try{
        std::string label = node["label"].as<std::string>();
    }catch(YAML::TypedBadConversion<std::string> &e){
        std::cout<<"label node is NULL"<<std::endl;
    }

    //修改
    node["mg"] = 2;

    //保存config为yaml文件
    std::ofstream fout("./test.yaml");
    fout << node;
    fout.close();
    return 0;
}
