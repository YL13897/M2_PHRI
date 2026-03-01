//Compilation: g++ main.cpp -L lib -l FLNL -lpthread -o Server

#include <iostream>
#include <chrono>

#include "FLNL.h"


int main()
{
    server *monServer;
    monServer=new server();

    if(monServer->Connect("192.168.1.100")!=0)
    {
        exit(-1);
    }


    std::vector<double> i({0.1, 42.0, 54.0});
    double t=0;
    std::vector<double> a(2);

    while(!monServer->IsConnected())
    {
        monServer->Reconnect();
        usleep(1000);

        auto t0 = std::chrono::steady_clock::now();
        while(monServer->IsConnected())
        {
            auto t1 = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::milli> t_ms = t1-t0;
            t = t_ms.count()/1000.;
            i[0]=t;

            if(t>3)
                monServer->Send(i);

            if(monServer->IsReceivedValues())
            {
                monServer->GetReceivedValues(a);
                std::cout << t << " =? " << a[0] << " " << t-a[0] << std::endl;
            }
            if(monServer->IsReceivedCmd())
            {
                std::string ss;
                monServer->GetReceivedCmd(ss, a);
                std::cout << ss << "( ";
                for(int i=0; i<a.size(); i++)
                    std::cout << a[i] << " ";
                std::cout << ")" << std::endl;
                if(ss == "move")
                {
                    monServer->Send("OK", {25});
                }
            }
            usleep(1000);

            //Randomly pause to test asynchronicity
            if(rand()%1000==1){
                std::cout << "Pause" <<std::endl;
                for(int i=0; i<50; i++)
                    usleep(100000);
            }
        }
    }

    delete monServer;
}
