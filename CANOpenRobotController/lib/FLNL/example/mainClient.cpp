//Compilation: g++ mainClient.cpp -L lib -l FLNL -lpthread -o Client
#include <iostream>
#include <chrono>

#include "FLNL.h"


int main()
{
	usleep(2000000);
    client *monClient;
    monClient=new client();
    if(monClient->Connect("192.168.7.2")!=0)
    {
        exit(-1);
    }

    std::vector<double> i({0,0,0});
    float m=0;
    std::vector<double> a(6);
    double t;
    auto t0 = std::chrono::steady_clock::now();

    while(monClient->IsConnected())
    {
        auto t1 = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> t_ms = t1-t0;
        t = t_ms.count()/1000.;
        i[0] = t;

        if(monClient->IsReceivedValues())
        {
            monClient->GetReceivedValues(a);
            std::cout << t << " =? " << a[0] << " " << t-a[0] << std::endl;
            /*for(int k=0; k<6; k++)
                std::cout << a[k] << "\t";
            std::cout << std::endl;*/
        }
        m+=0.00;
        i[2]=-4.*a[5];
        monClient->Send(i);

        if(monClient->IsReceivedCmd())
        {
            std::string ss;
            monClient->GetReceivedCmd(ss, a);
            std::cout << ss << std::endl;
        }

        if(rand()%1000==1)
            monClient->Send("GTNS", {2.56});

        usleep(10);
    }
    std::cout <<  "Closing..." << std::endl;

    delete monClient;

    return 0;
}
