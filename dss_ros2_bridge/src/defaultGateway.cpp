#include "defaultGateway.h"
#include <memory>
#include <string>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <fstream>


std::string getDefaultGateway()
{
    std::ifstream file("/proc/net/route");
    if (!file.is_open()) {
        return "";
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);

        std::string iface, destination, gatewayHex;
        unsigned flags, refcnt, use, metric, mask, mtu, win, irtt;

        if (!(iss >> iface >> destination >> gatewayHex >> flags >> refcnt >> use
              >> metric >> mask >> mtu >> win >> irtt)) {
            continue;
        }

        // Destination 00000000 = default route
        if (destination == "00000000") {
            unsigned long gatewayUL = std::stoul(gatewayHex, nullptr, 16);

            struct in_addr addr;
            addr.s_addr = static_cast<uint32_t>(gatewayUL);

            return inet_ntoa(addr);
        }
    }

    return "";
}
