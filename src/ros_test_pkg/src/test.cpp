#include <iostream>
#include <algorithm>

#include <crazyflie_cpp/Crazyflie.h>
#include <crazyflie_cpp/crtp.h>
#include <crazyflie_cpp/Crazyradio.h>


#include <unistd.h>
#include <cstdlib>

#include <chrono>
#include <ctime>    

unsigned int microseconds = 1000; 


int main(int argc, char **argv)
{
    std::string defaultUri("radio://0/80/2M/E7E7E7E7E7");

    std::string parameter("extModeControl.motVal1");
    std::string parameter2("extModeControl.motVal2");
    std::string parameter3("extModeControl.motVal3");

    std::cout << argv[0] << " " << argv[1]<< "\n";
    
    int angle = atoi(argv[1]);
    try
    {
        Crazyflie cf(defaultUri);

        CrazyflieBroadcaster cfb(defaultUri);
        // usleep(microseconds);

        float x, y, z;
        float qx, qy, qz, qw;
        x = 0;
        z = 0;
        qx = 0;
        qy = 0;
        qz = 0;
        qw = 1;

        auto all_start = std::chrono::system_clock::now();
        auto start = std::chrono::system_clock::now();
        auto mid = std::chrono::system_clock::now();
        
        std::chrono::duration<double> elapsed_seconds_mid = mid-start;
        for(int i =0; i<100; i++){
            start = std::chrono::system_clock::now();
            cf.sendExternalPoseUpdate(x, y, z, qx, qy, qz, qw);
            mid = std::chrono::system_clock::now();
            elapsed_seconds_mid = mid-start;
            std::cout << "sending time!" << elapsed_seconds_mid.count() << "s"<< "\n";
        }
        elapsed_seconds_mid = mid-all_start;
        std::cout << "All time!" << elapsed_seconds_mid.count() << "s"<< "\n";
        std::cout << "All good!" << "\n";
        // cfb.stop(1);
        // auto end = std::chrono::system_clock::now();

// // ---------------------------------------
//         auto pos = parameter.find(".");
//         const char* group = parameter.substr(0, pos).c_str();
//         const char* name = parameter.substr(pos+1).c_str();

//         auto start = std::chrono::system_clock::now();
//         cf.setParamByName<uint16_t>(group, name, angle);
//         auto mid = std::chrono::system_clock::now();

//         pos = parameter2.find(".");
//         group = parameter2.substr(0, pos).c_str();
//         name = parameter2.substr(pos+1).c_str();
//         cf.setParamByName<uint16_t>(group, name, angle);

//         pos = parameter3.find(".");
//         group = parameter3.substr(0, pos).c_str();
//         name = parameter3.substr(pos+1).c_str();
//         cf.setParamByName<uint16_t>(group, name, angle);
//         auto end = std::chrono::system_clock::now();

// // -----------------------------------------

        // auto start = std::chrono::system_clock::now();
        // cf.requestParamToc();
        // auto mid = std::chrono::system_clock::now();

        // std::for_each(cf.paramsBegin(), cf.paramsEnd(),
        // [](const Crazyflie::ParamTocEntry& entry)
        // {
        //     std::cout << entry.group << "." << entry.name << " (";
        //     switch (entry.type) {
        //     case Crazyflie::ParamTypeUint8:
        //     std::cout << "uint8";
        //     break;
        //     case Crazyflie::ParamTypeInt8:
        //     std::cout << "int8";
        //     break;
        //     case Crazyflie::ParamTypeUint16:
        //     std::cout << "uint16";
        //     break;
        //     case Crazyflie::ParamTypeInt16:
        //     std::cout << "int16";
        //     break;
        //     case Crazyflie::ParamTypeUint32:
        //     std::cout << "uint32";
        //     break;
        //     case Crazyflie::ParamTypeInt32:
        //     std::cout << "int32";
        //     break;
        //     case Crazyflie::ParamTypeFloat:
        //     std::cout << "float";
        //     break;
        //     }
        //     if (entry.readonly) {
        //     std::cout << ", readonly";
        //     }
        //     std::cout << ")";
        //     std::cout << std::endl;
        // }
        // );



        // auto end = std::chrono::system_clock::now();
        // std::chrono::duration<double> elapsed_seconds = end-start;
        // std::cout << "All time!" << elapsed_seconds.count() << "s"<< "\n";
        
    }
    catch(std::exception& e)
    {
        std::cout << "Error!" << "\n";
    }
    
    std::cout << "Done!" << "\n";
    return 0;
}