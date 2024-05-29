/*
 * Copyright (c) 2017 University of Padova
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

#include "lorawan-mac-helper.h"

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/log.h"
#include "ns3/lora-net-device.h"
#include "ns3/random-variable-stream.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <bits/stdc++.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>

int a=0,b=0,c=0,d=0,e=0;
struct Point {
    double x;
    double y;
};

// Function to calculate Euclidean distance between two points
double euclidean_distance(Point p1, Point p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// Function to find the nearest 6 coordinates to a given coordinate
std::vector<Point> find_nearest_6(const std::vector<Point>& coordinates, int current_index) {
    const Point& current_coord = coordinates[current_index];
    std::vector<std::pair<int, double>> distances;
    distances.reserve(coordinates.size() - 1);
    for (int i = 0; i < coordinates.size(); ++i) {
        if (i != current_index) {
            distances.emplace_back(i, euclidean_distance(current_coord, coordinates[i]));
        }
    }
    std::sort(distances.begin(), distances.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    });
    std::vector<Point> nearest_coords;
    nearest_coords.reserve(6);
    for (int i = 0; i < std::min(6, static_cast<int>(distances.size())); ++i) {
        nearest_coords.push_back(coordinates[distances[i].first]);
    }
    return nearest_coords;
}

pair<string,string> allocate_sf(double distance) {
    if (distance <= 1200) {
        return {"7", "8"};
    } else if (distance <= 2400) {
        return {"8", "9"};
    } else if (distance <= 3600) {
        return {"9", "10"};
    } else if (distance <= 4800) {
        return {"10", "11"};
    } else if (distance <= 6000) {
        return {"11", "12"};
    } else {
        return {"12","a"}; // Return an empty string for the second value
    }
}

string allocate_sf_sector(double angle,double dist) {
    // Convert angle to sector number (0 to 5)
    int sector = static_cast<int>((angle + 360.0) / 60.0) % 6;

    // Define SFs for each distance region
    vector<vector<string>> distance_SFs = {
        
        {"7", "8"},
        {"8", "9"},
        {"9", "10"},
        {"10", "11"},
        {"11", "12"},
        {"12", "12"} // Empty string for last distance region
    };

    // Get available SFs for the current distance region
    vector<string> available_SFs;
    
    
    if(dist<=1200)
    available_SFs=distance_SFs[0];
    else if(dist<=2400)
    available_SFs=distance_SFs[1];
    else if(dist<=3600)
    available_SFs=distance_SFs[2];
    else if(dist<=4800)
    available_SFs=distance_SFs[3];
    else if(dist<=6000)
    available_SFs=distance_SFs[4];
    else
    available_SFs=distance_SFs[5];

    // Allocate SFs to odd and even sectors alternately
    string selected_SF;
    if (sector % 2 == 0) { // Even sector
        selected_SF = available_SFs[1];
    } else { // Odd sector
        selected_SF = available_SFs[0];
    }

    // Return the selected SF
    return selected_SF; // Second value is empty string as before
}




vector<Point> points;
std::unordered_map<std::string, std::vector<Point>> cluster_centroids;




using namespace std;

vector<vector<string>> da;
vector<string> v;

namespace ns3
{
namespace lorawan
{

NS_LOG_COMPONENT_DEFINE("LorawanMacHelper");

LorawanMacHelper::LorawanMacHelper()
    : m_region(LorawanMacHelper::EU)
{
}

vector<vector<string>> LorawanMacHelper::GetData(){
return da;
}

void
LorawanMacHelper::Set(std::string name, const AttributeValue& v)
{
    m_mac.Set(name, v);
}

void
LorawanMacHelper::SetDeviceType(enum DeviceType dt)
{
    NS_LOG_FUNCTION(this << dt);
    switch (dt)
    {
    case GW:
        m_mac.SetTypeId("ns3::GatewayLorawanMac");
        break;
    case ED_A:
        m_mac.SetTypeId("ns3::ClassAEndDeviceLorawanMac");
        break;
    }
    m_deviceType = dt;
}

void
LorawanMacHelper::SetAddressGenerator(Ptr<LoraDeviceAddressGenerator> addrGen)
{
    NS_LOG_FUNCTION(this);

    m_addrGen = addrGen;
}

void
LorawanMacHelper::SetRegion(enum LorawanMacHelper::Regions region)
{
    m_region = region;
}

Ptr<LorawanMac>
LorawanMacHelper::Create(Ptr<Node> node, Ptr<NetDevice> device) const
{
    Ptr<LorawanMac> mac = m_mac.Create<LorawanMac>();
    mac->SetDevice(device);

    // If we are operating on an end device, add an address to it
    if (m_deviceType == ED_A && m_addrGen)
    {
        mac->GetObject<ClassAEndDeviceLorawanMac>()->SetDeviceAddress(m_addrGen->NextAddress());
    }

    // Add a basic list of channels based on the region where the device is
    // operating
    if (m_deviceType == ED_A)
    {
        Ptr<ClassAEndDeviceLorawanMac> edMac = mac->GetObject<ClassAEndDeviceLorawanMac>();
        switch (m_region)
        {
        case LorawanMacHelper::EU: {
            ConfigureForEuRegion(edMac);
            break;
        }
        case LorawanMacHelper::SingleChannel: {
            ConfigureForSingleChannelRegion(edMac);
            break;
        }
        case LorawanMacHelper::ALOHA: {
            ConfigureForAlohaRegion(edMac);
            break;
        }
        default: {
            NS_LOG_ERROR("This region isn't supported yet!");
            break;
        }
        }
    }
    else
    {
        Ptr<GatewayLorawanMac> gwMac = mac->GetObject<GatewayLorawanMac>();
        switch (m_region)
        {
        case LorawanMacHelper::EU: {
            ConfigureForEuRegion(gwMac);
            break;
        }
        case LorawanMacHelper::SingleChannel: {
            ConfigureForSingleChannelRegion(gwMac);
            break;
        }
        case LorawanMacHelper::ALOHA: {
            ConfigureForAlohaRegion(gwMac);
            break;
        }
        default: {
            NS_LOG_ERROR("This region isn't supported yet!");
            break;
        }
        }
    }
    return mac;
}

void
LorawanMacHelper::ConfigureForAlohaRegion(Ptr<ClassAEndDeviceLorawanMac> edMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    ApplyCommonAlohaConfigurations(edMac);

    /////////////////////////////////////////////////////
    // TxPower -> Transmission power in dBm conversion //
    /////////////////////////////////////////////////////
    edMac->SetTxDbmForTxPower(std::vector<double>{16, 14, 12, 10, 8, 6, 4, 2});

    ////////////////////////////////////////////////////////////
    // Matrix to know which DataRate the GW will respond with //
    ////////////////////////////////////////////////////////////
    LorawanMac::ReplyDataRateMatrix matrix = {{{{0, 0, 0, 0, 0, 0}},
                                               {{1, 0, 0, 0, 0, 0}},
                                               {{2, 1, 0, 0, 0, 0}},
                                               {{3, 2, 1, 0, 0, 0}},
                                               {{4, 3, 2, 1, 0, 0}},
                                               {{5, 4, 3, 2, 1, 0}},
                                               {{6, 5, 4, 3, 2, 1}},
                                               {{7, 6, 5, 4, 3, 2}}}};
    edMac->SetReplyDataRateMatrix(matrix);

    /////////////////////
    // Preamble length //
    /////////////////////
    edMac->SetNPreambleSymbols(8);

    //////////////////////////////////////
    // Second receive window parameters //
    //////////////////////////////////////
    edMac->SetSecondReceiveWindowDataRate(0);
    edMac->SetSecondReceiveWindowFrequency(869.525);
}

void
LorawanMacHelper::ConfigureForAlohaRegion(Ptr<GatewayLorawanMac> gwMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    ///////////////////////////////
    // ReceivePath configuration //
    ///////////////////////////////
    Ptr<GatewayLoraPhy> gwPhy =
        gwMac->GetDevice()->GetObject<LoraNetDevice>()->GetPhy()->GetObject<GatewayLoraPhy>();

    ApplyCommonAlohaConfigurations(gwMac);

    if (gwPhy) // If cast is successful, there's a GatewayLoraPhy
    {
        NS_LOG_DEBUG("Resetting reception paths");
        gwPhy->ResetReceptionPaths();

        int receptionPaths = 0;
        int maxReceptionPaths = 1;
        while (receptionPaths < maxReceptionPaths)
        {
            gwPhy->GetObject<GatewayLoraPhy>()->AddReceptionPath();
            receptionPaths++;
        }
        gwPhy->AddFrequency(868.1);
    }
}

void
LorawanMacHelper::ApplyCommonAlohaConfigurations(Ptr<LorawanMac> lorawanMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    //////////////
    // SubBands //
    //////////////

    LogicalLoraChannelHelper channelHelper;
    channelHelper.AddSubBand(868, 868.6, 1, 14);

    //////////////////////
    // Default channels //
    //////////////////////
    Ptr<LogicalLoraChannel> lc1 = CreateObject<LogicalLoraChannel>(868.1, 0, 5);
    channelHelper.AddChannel(lc1);

    lorawanMac->SetLogicalLoraChannelHelper(channelHelper);

    ///////////////////////////////////////////////
    // DataRate -> SF, DataRate -> Bandwidth     //
    // and DataRate -> MaxAppPayload conversions //
    ///////////////////////////////////////////////
    lorawanMac->SetSfForDataRate(std::vector<uint8_t>{12, 11, 10, 9, 8, 7, 7});
    lorawanMac->SetBandwidthForDataRate(
        std::vector<double>{125000, 125000, 125000, 125000, 125000, 125000, 250000});
    lorawanMac->SetMaxAppPayloadForDataRate(
        std::vector<uint32_t>{59, 59, 59, 123, 230, 230, 230, 230});
}

void
LorawanMacHelper::ConfigureForEuRegion(Ptr<ClassAEndDeviceLorawanMac> edMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    ApplyCommonEuConfigurations(edMac);

    /////////////////////////////////////////////////////
    // TxPower -> Transmission power in dBm conversion //
    /////////////////////////////////////////////////////
    edMac->SetTxDbmForTxPower(std::vector<double>{16, 14, 12, 10, 8, 6, 4, 2});

    ////////////////////////////////////////////////////////////
    // Matrix to know which DataRate the GW will respond with //
    ////////////////////////////////////////////////////////////
    LorawanMac::ReplyDataRateMatrix matrix = {{{{0, 0, 0, 0, 0, 0}},
                                               {{1, 0, 0, 0, 0, 0}},
                                               {{2, 1, 0, 0, 0, 0}},
                                               {{3, 2, 1, 0, 0, 0}},
                                               {{4, 3, 2, 1, 0, 0}},
                                               {{5, 4, 3, 2, 1, 0}},
                                               {{6, 5, 4, 3, 2, 1}},
                                               {{7, 6, 5, 4, 3, 2}}}};
    edMac->SetReplyDataRateMatrix(matrix);

    /////////////////////
    // Preamble length //
    /////////////////////
    edMac->SetNPreambleSymbols(8);

    //////////////////////////////////////
    // Second receive window parameters //
    //////////////////////////////////////
    edMac->SetSecondReceiveWindowDataRate(0);
    edMac->SetSecondReceiveWindowFrequency(869.525);
}

void
LorawanMacHelper::ConfigureForEuRegion(Ptr<GatewayLorawanMac> gwMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    ///////////////////////////////
    // ReceivePath configuration //
    ///////////////////////////////
    Ptr<GatewayLoraPhy> gwPhy =
        gwMac->GetDevice()->GetObject<LoraNetDevice>()->GetPhy()->GetObject<GatewayLoraPhy>();

    ApplyCommonEuConfigurations(gwMac);

    if (gwPhy) // If cast is successful, there's a GatewayLoraPhy
    {
        NS_LOG_DEBUG("Resetting reception paths");
        gwPhy->ResetReceptionPaths();

        std::vector<double> frequencies;
        frequencies.push_back(868.1);
        frequencies.push_back(868.3);
        frequencies.push_back(868.5);

        for (auto& f : frequencies)
        {
            gwPhy->AddFrequency(f);
        }

        int receptionPaths = 0;
        int maxReceptionPaths = 8;
        while (receptionPaths < maxReceptionPaths)
        {
            gwPhy->GetObject<GatewayLoraPhy>()->AddReceptionPath();
            receptionPaths++;
        }
    }
}

void
LorawanMacHelper::ApplyCommonEuConfigurations(Ptr<LorawanMac> lorawanMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    //////////////
    // SubBands //
    //////////////

    LogicalLoraChannelHelper channelHelper;
    channelHelper.AddSubBand(868, 868.6, 0.01, 14);
    channelHelper.AddSubBand(868.7, 869.2, 0.001, 14);
    channelHelper.AddSubBand(869.4, 869.65, 0.1, 27);

    //////////////////////
    // Default channels //
    //////////////////////
    Ptr<LogicalLoraChannel> lc1 = CreateObject<LogicalLoraChannel>(868.1, 0, 5);
    Ptr<LogicalLoraChannel> lc2 = CreateObject<LogicalLoraChannel>(868.3, 0, 5);
    Ptr<LogicalLoraChannel> lc3 = CreateObject<LogicalLoraChannel>(868.5, 0, 5);
    channelHelper.AddChannel(lc1);
    channelHelper.AddChannel(lc2);
    channelHelper.AddChannel(lc3);

    lorawanMac->SetLogicalLoraChannelHelper(channelHelper);

    ///////////////////////////////////////////////
    // DataRate -> SF, DataRate -> Bandwidth     //
    // and DataRate -> MaxAppPayload conversions //
    ///////////////////////////////////////////////
    lorawanMac->SetSfForDataRate(std::vector<uint8_t>{12, 11, 10, 9, 8, 7, 7});
    lorawanMac->SetBandwidthForDataRate(
        std::vector<double>{125000, 125000, 125000, 125000, 125000, 125000, 250000});
    lorawanMac->SetMaxAppPayloadForDataRate(
        std::vector<uint32_t>{59, 59, 59, 123, 230, 230, 230, 230});
}

///////////////////////////////

void
LorawanMacHelper::ConfigureForSingleChannelRegion(Ptr<ClassAEndDeviceLorawanMac> edMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    ApplyCommonSingleChannelConfigurations(edMac);

    /////////////////////////////////////////////////////
    // TxPower -> Transmission power in dBm conversion //
    /////////////////////////////////////////////////////
    edMac->SetTxDbmForTxPower(std::vector<double>{16, 14, 12, 10, 8, 6, 4, 2});

    ////////////////////////////////////////////////////////////
    // Matrix to know which DataRate the GW will respond with //
    ////////////////////////////////////////////////////////////
    LorawanMac::ReplyDataRateMatrix matrix = {{{{0, 0, 0, 0, 0, 0}},
                                               {{1, 0, 0, 0, 0, 0}},
                                               {{2, 1, 0, 0, 0, 0}},
                                               {{3, 2, 1, 0, 0, 0}},
                                               {{4, 3, 2, 1, 0, 0}},
                                               {{5, 4, 3, 2, 1, 0}},
                                               {{6, 5, 4, 3, 2, 1}},
                                               {{7, 6, 5, 4, 3, 2}}}};
    edMac->SetReplyDataRateMatrix(matrix);

    /////////////////////
    // Preamble length //
    /////////////////////
    edMac->SetNPreambleSymbols(8);

    //////////////////////////////////////
    // Second receive window parameters //
    //////////////////////////////////////
    edMac->SetSecondReceiveWindowDataRate(0);
    edMac->SetSecondReceiveWindowFrequency(869.525);
}

void
LorawanMacHelper::ConfigureForSingleChannelRegion(Ptr<GatewayLorawanMac> gwMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    ///////////////////////////////
    // ReceivePath configuration //
    ///////////////////////////////
    Ptr<GatewayLoraPhy> gwPhy =
        gwMac->GetDevice()->GetObject<LoraNetDevice>()->GetPhy()->GetObject<GatewayLoraPhy>();

    ApplyCommonEuConfigurations(gwMac);

    if (gwPhy) // If cast is successful, there's a GatewayLoraPhy
    {
        NS_LOG_DEBUG("Resetting reception paths");
        gwPhy->ResetReceptionPaths();

        std::vector<double> frequencies;
        frequencies.push_back(868.1);

        for (auto& f : frequencies)
        {
            gwPhy->AddFrequency(f);
        }

        int receptionPaths = 0;
        int maxReceptionPaths = 8;
        while (receptionPaths < maxReceptionPaths)
        {
            gwPhy->GetObject<GatewayLoraPhy>()->AddReceptionPath();
            receptionPaths++;
        }
    }
}

void
LorawanMacHelper::ApplyCommonSingleChannelConfigurations(Ptr<LorawanMac> lorawanMac) const
{
    NS_LOG_FUNCTION_NOARGS();

    //////////////
    // SubBands //
    //////////////

    LogicalLoraChannelHelper channelHelper;
    channelHelper.AddSubBand(868, 868.6, 0.01, 14);
    channelHelper.AddSubBand(868.7, 869.2, 0.001, 14);
    channelHelper.AddSubBand(869.4, 869.65, 0.1, 27);

    //////////////////////
    // Default channels //
    //////////////////////
    Ptr<LogicalLoraChannel> lc1 = CreateObject<LogicalLoraChannel>(868.1, 0, 5);
    channelHelper.AddChannel(lc1);

    lorawanMac->SetLogicalLoraChannelHelper(channelHelper);

    ///////////////////////////////////////////////
    // DataRate -> SF, DataRate -> Bandwidth     //
    // and DataRate -> MaxAppPayload conversions //
    ///////////////////////////////////////////////
    lorawanMac->SetSfForDataRate(std::vector<uint8_t>{12, 11, 10, 9, 8, 7, 7});
    lorawanMac->SetBandwidthForDataRate(
        std::vector<double>{125000, 125000, 125000, 125000, 125000, 125000, 250000});
    lorawanMac->SetMaxAppPayloadForDataRate(
        std::vector<uint32_t>{59, 59, 59, 123, 230, 230, 230, 230});
}

/*std::vector<int>
LorawanMacHelper::SetSpreadingFactorsUp(const string& filePath, int NDevices, int radius, const string& model_name,NodeContainer endDevices,
                                        NodeContainer gateways,
                                        Ptr<LoraChannel> channel,int i)
{
    NS_LOG_FUNCTION_NOARGS();
    
    std::vector<int> sfQuantity(7, 0);
    
    ifstream file(filePath); // Open the CSV file
    if (!file.is_open()) {
        cerr << "Error: Unable to open file: " << filePath << endl;
        return sfQuantity;
    }

    string line;
    bool found = false;
    vector<string> headers;
    unordered_map<string, int> columnIndices;

    // Read the headers
    if (getline(file, line)) {
        stringstream ss(line);
        string cell;
        while (getline(ss, cell, ',')) {
            headers.push_back(cell);
            columnIndices[cell] = headers.size() - 1; // Store column indices by column name
        }
        found = true;
    }

    while (getline(file, line)) { // Iterate through each line in the file
        stringstream ss(line);
        string cell;

        if (found) {
            // Read values from subsequent lines
            vector<string> values;
            while (getline(ss, cell, ',')) {
                values.push_back(cell);
            }

            // Check if the row matches the specified endDevices and radius
            if (stoi(values[columnIndices["Nodes"]]) == NDevices && stoi(values[columnIndices["Radius"]]) == radius) {
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        
        Ptr<LoraNetDevice> loraNetDevice_1 = object->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<LoraPhy> phy = loraNetDevice_1->GetPhy();
        
        
        ostringstream oss,oss1,oss2,oss3,oss4,oss5,oss6,oss7,oss8,oss9;
        oss<<object;
        v.push_back(oss.str());
        
        Ptr<MobilityModel> position = (*j)->GetObject<MobilityModel>();
        Vector pos = position->GetPosition();
        
        oss1<<pos.x;
        v.push_back(oss1.str());
        oss.clear();
        
        oss2<<pos.y;
        v.push_back(oss2.str());
        oss.clear(); 

        oss3<<pos.z;
        v.push_back(oss3.str());
        oss.clear();
      
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);


        Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();
        NS_ASSERT(loraNetDevice);
        
        
        Ptr<ClassAEndDeviceLorawanMac> mac =
            loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
        NS_ASSERT(mac);


        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();


        // Assume devices transmit at 14 dBm
        double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm
            
            

            if (currentRxPower > highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = curr->GetObject<MobilityModel>();
                highestRxPower = currentRxPower;
              
            }
            
            
        }
        
        
        oss5<<bestGateway;
        v.push_back(oss5.str());

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        oss8<<rxPower;
        v.push_back(oss8.str());
        oss.clear();

        // Get the ED sensitivity
        Ptr<EndDeviceLoraPhy> edPhy = loraNetDevice->GetPhy()->GetObject<EndDeviceLoraPhy>();
        const double* edSensitivity = edPhy->sensitivity;
        
        

        if (rxPower > *edSensitivity)
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;
            v.push_back("7");
        }
        else i
        f (rxPower > *(edSensitivity + 1))
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;
            v.push_back("8");
        }
        else if (rxPower > *(edSensitivity + 2))
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;
            v.push_back("9");
        }
        else if (rxPower > *(edSensitivity + 3))
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;
            v.push_back("10");
        }
        else if (rxPower > *(edSensitivity + 4))
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;
            v.push_back("11");
        }
        else if (rxPower > *(edSensitivity + 5))
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;
            v.push_back("12");
        }
        else // Device is out of range. Assign SF12.
        {
            // NS_LOG_DEBUG ("Device out of range");
            mac->SetDataRate(0);
            sfQuantity[6] = sfQuantity[6] + 1;
            // NS_LOG_DEBUG ("sfQuantity[6] = " << sfQuantity[6]);
            v.push_back("12");
        }

        /*

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = gatewayNetDevice->GetObject<LoraNetDevice> ();
        Ptr<GatewayLoraPhy> gatewayPhy = gatewayLoraNetDevice->GetPhy ()->GetObject<GatewayLoraPhy>
        (); const double *gwSensitivity = gatewayPhy->sensitivity;

        if(rxPower > *gwSensitivity)
          {
            mac->SetDataRate (5);
            sfQuantity[0] = sfQuantity[0] + 1;

          }
        else if (rxPower > *(gwSensitivity+1))
          {
            mac->SetDataRate (4);
            sfQuantity[1] = sfQuantity[1] + 1;

          }
        else if (rxPower > *(gwSensitivity+2))
          {
            mac->SetDataRate (3);
            sfQuantity[2] = sfQuantity[2] + 1;

          }
        else if (rxPower > *(gwSensitivity+3))
          {
            mac->SetDataRate (2);
            sfQuantity[3] = sfQuantity[3] + 1;
          }
        else if (rxPower > *(gwSensitivity+4))
          {
            mac->SetDataRate (1);
            sfQuantity[4] = sfQuantity[4] + 1;
          }
        else if (rxPower > *(gwSensitivity+5))
          {
            mac->SetDataRate (0);
            sfQuantity[5] = sfQuantity[5] + 1;

          }
        else // Device is out of range. Assign SF12.
          {
            mac->SetDataRate (0);
            sfQuantity[6] = sfQuantity[6] + 1;

          }
          */
          
          /*if (values[columnIndices["SF_" + model_name]]=="7.0")
                                    {
                                        mac->SetDataRate(5);
                                        sfQuantity[0] = sfQuantity[0] + 1;
                                        v.push_back("7");
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="8.0")
                                    {
                                        mac->SetDataRate(4);
                                        sfQuantity[1] = sfQuantity[1] + 1;
                                        v.push_back("8");
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="9.0")
                                    {
                                        mac->SetDataRate(3);
                                        sfQuantity[2] = sfQuantity[2] + 1;
                                        v.push_back("9");
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="10.0")
                                    {
                                        mac->SetDataRate(2);
                                        sfQuantity[3] = sfQuantity[3] + 1;
                                        v.push_back("10");
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="11.0")
                                    {
                                        mac->SetDataRate(1);
                                        sfQuantity[4] = sfQuantity[4] + 1;
                                        v.push_back("11");
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="12.0")
                                    {
                                        mac->SetDataRate(0);
                                        sfQuantity[5] = sfQuantity[5] + 1;
                                        v.push_back("12");
                                    }
                                    else // Device is out of range. Assign SF12.
                                    {
                                        // NS_LOG_DEBUG ("Device out of range");
                                        mac->SetDataRate(0);
                                        sfQuantity[6] = sfQuantity[6] + 1;
                                        // NS_LOG_DEBUG ("sfQuantity[6] = " << sfQuantity[6]);
                                        v.push_back("12");
                                    }
          
          
          
          d.push_back(v);
          v.clear();

    }}}} // end loop on nodes

    return sfQuantity;*/


    /*NS_LOG_FUNCTION_NOARGS();
    
    std::vector<int> sfQuantity(7, 0);
    
    ifstream file(filePath); // Open the CSV file
    if (!file.is_open()) {
        cerr << "Error: Unable to open file: " << filePath << endl;
        return sfQuantity;
    }

    file.seekg(0); // Move the file pointer to the beginning

    // Skip lines until the desired row
    for (long long int j = 1; j < i; ++j) { // Adjust loop to start from 1
        if (!getline(file, line)) {
            cerr << "Error: File has fewer lines than specified index." << endl;
            return sfQuantity;
        }
    }

    // Now the file pointer is at the ith row, proceed with processing it
    string line;
    if (!getline(file, line)) {
        cerr << "Error: Unable to read line: " << i << " from file: " << filePath << endl;
        return sfQuantity;
    }

    // Process the line as before
    stringstream ss(line);
    string cell;

    // Read values from the line
    vector<string> values;
    while (getline(ss, cell, ',')) {
        values.push_back(cell);
    }

    // Check if the row matches the specified endDevices and radius
    /*if (stoi(values[columnIndices["Nodes"]]) == NDevices && stoi(values[columnIndices["Radius"]]) == radius) {
        for (auto j = endDevices.Begin(); j != endDevices.End(); ++j) {
                NS_LOG_FUNCTION_NOARGS();
    
    std::vector<int> sfQuantity(7, 0);
    
    ifstream file(filePath); // Open the CSV file
    if (!file.is_open()) {
        cerr << "Error: Unable to open file: " << filePath << endl;
        return sfQuantity;
    }

    file.seekg(0); // Move the file pointer to the beginning

    // Skip lines until the desired row
    for (long long int j = 1; j < i; ++j) { // Adjust loop to start from 1
        if (!getline(file, line)) {
            cerr << "Error: File has fewer lines than specified index." << endl;
            return sfQuantity;
        }
    }

    // Now the file pointer is at the ith row, proceed with processing it
    string line;
    if (!getline(file, line)) {
        cerr << "Error: Unable to read line: " << i << " from file: " << filePath << endl;
        return sfQuantity;
    }

    // Process the line as before
    stringstream ss(line);
    string cell;

    // Read values from the line
    vector<string> values;
    while (getline(ss, cell, ',')) {
        values.push_back(cell);
    }*/

    // Check if the row matches the specified endDevices and radius
    /*if (stoi(values[columnIndices["Nodes"]]) == NDevices && stoi(values[columnIndices["Radius"]]) == radius) {
        for (aut8o j = endDevices.Begin(); j != endDevices.End(); ++j) {
            // Process each end device
            // ...
            if (values[columnIndices["SF_" + model_name]]=="7.0")
                                    {
                                        mac->SetDataRate(5);
                                        sfQuantity[0] = sfQuantity[0] + 1;
                                        v.push_back("7");
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="8.0")
                                    {
                                        mac->SetDataRate(4);
                                        sfQuantity[1] = sfQuantity[1] + 1;
                                        v.push_back("8");
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="9.0")
                                    {
                                        mac->SetDataRate(3);
                                        sfQuantity[2] = sfQuantity[2] + 1;
                                        v.push_back("9");
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="10.0")
                                    {
                                        mac->SetDataRate(2);
                                        sfQuantity[3] = sfQuantity[3] + 1;
                                        v.push_back("10");
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="11.0")
                                    {
                                        mac->SetDataRate(1);
                                        sfQuantity[4] = sfQuantity[4] + 1;
                                        v.push_back("11");
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="12.0")
                                    {
                                        mac->SetDataRate(0);
                                        sfQuantity[5] = sfQuantity[5] + 1;
                                        v.push_back("12");
                                    }
                                    else // Device is out of range. Assign SF12.
                                    {
                                        // NS_LOG_DEBUG ("Device out of range");
                                        mac->SetDataRate(0);
                                        sfQuantity[6] = sfQuantity[6] + 1;
                                        // NS_LOG_DEBUG ("sfQuantity[6] = " << sfQuantity[6]);
                                        v.push_back("12");
                                    }
          
          
          
          d.push_back(v);
          v.clear();
        }
    }

    return sfQuantity;
        }
    }*/
    






/*void readPredictedSFResult(const string& filePath, int NDevices, int radius, const string& model_name,NodeContainer endDevices,
                                        NodeContainer gateways,
                                        Ptr<LoraChannel> channel) {
    NS_LOG_FUNCTION_NOARGS();
    vector<int> sfQuantity(7, 0);
    ifstream file(filePath); // Open the CSV file
    if (!file.is_open()) {
        cerr << "Error: Unable to open file: " << filePath << endl;
        return;
    }

    string line;
    bool found = false;
    vector<string> headers;
    unordered_map<string, int> columnIndices;

    // Read the headers
    if (getline(file, line)) {
        stringstream ss(line);
        string cell;
        while (getline(ss, cell, ',')) {
            headers.push_back(cell);
            columnIndices[cell] = headers.size() - 1; // Store column indices by column name
        }
        found = true;
    }

    while (getline(file, line)) { // Iterate through each line in the file
        stringstream ss(line);
        string cell;

        if (found) {
            // Read values from subsequent lines
            vector<string> values;
            while (getline(ss, cell, ',')) {
                values.push_back(cell);
            }

            // Check if the row matches the specified endDevices and radius
            if (stoi(values[columnIndices["Nodes"]]) == NDevices && stoi(values[columnIndices["Radius"]]) == radius) {

                                        for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
                                {
                                    Ptr<Node> object = *j;
                                    
                                    Ptr<LoraNetDevice> loraNetDevice_1 = object->GetDevice(0)->GetObject<LoraNetDevice>();
                                    Ptr<LoraPhy> phy = loraNetDevice_1->GetPhy();
                                    
                                    
                                    Ptr<MobilityModel> position = (*j)->GetObject<MobilityModel>();
                                    Vector pos = position->GetPosition();
                                    
                                  
                                    NS_ASSERT(position);
                                    Ptr<NetDevice> netDevice = object->GetDevice(0);


                                    Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();
                                    NS_ASSERT(loraNetDevice);
                                    
                                    
                                    Ptr<ClassAEndDeviceLorawanMac> mac =
                                        loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                                    NS_ASSERT(mac);


                                    if (values[columnIndices["SF_" + model_name]]=="7.0")
                                    {
                                        mac->SetDataRate(5);
                                        sfQuantity[0] = sfQuantity[0] + 1;
                                        
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="8.0")
                                    {
                                        mac->SetDataRate(4);
                                        sfQuantity[1] = sfQuantity[1] + 1;
                                        
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="9.0")
                                    {
                                        mac->SetDataRate(3);
                                        sfQuantity[2] = sfQuantity[2] + 1;
                                        
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="10.0")
                                    {
                                        mac->SetDataRate(2);
                                        sfQuantity[3] = sfQuantity[3] + 1;
                                        
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="11.0")
                                    {
                                        mac->SetDataRate(1);
                                        sfQuantity[4] = sfQuantity[4] + 1;
                                        
                                    }
                                    else if (values[columnIndices["SF_" + model_name]]=="12.0")
                                    {
                                        mac->SetDataRate(0);
                                        sfQuantity[5] = sfQuantity[5] + 1;
                                        
                                    }
                                    else // Device is out of range. Assign SF12.
                                    {
                                        // NS_LOG_DEBUG ("Device out of range");
                                        mac->SetDataRate(0);
                                        sfQuantity[6] = sfQuantity[6] + 1;
                                        // NS_LOG_DEBUG ("sfQuantity[6] = " << sfQuantity[6]);
                                        
                                    }
            }
        }
    }

    file.close(); // Close the file
    
}}*/

std::vector<int> LorawanMacHelper::SetSpreadingFactorsUp(const string& filePath, int NDevices, int radius, const string& model_name, NodeContainer endDevices,
                                                          NodeContainer gateways, Ptr<LoraChannel> channel, int i) {
    NS_LOG_FUNCTION_NOARGS();
    
    std::vector<int> sfQuantity(7, 0);
    
                
                
                int k=0;
                for (auto j = endDevices.Begin(); j != endDevices.End(); ++j) {
                Ptr<Node> object = *j;
                Ptr<MobilityModel> position = (*j)->GetObject<MobilityModel>();
                    Vector pos = position->GetPosition();
                    points.push_back({pos.x,pos.y});
                }
                for (auto j = endDevices.Begin(); j != endDevices.End(); ++j) {
                Ptr<Node> object = *j;
                    
                    Ptr<LoraNetDevice> loraNetDevice_1 = object->GetDevice(0)->GetObject<LoraNetDevice>();
                    Ptr<LoraPhy> phy = loraNetDevice_1->GetPhy();
                    
                    ostringstream oss, oss1, oss2, oss3, oss4, oss5, oss6, oss7, oss8, oss9;
                    oss << object;
                    v.push_back(oss.str());
                    
                    Ptr<MobilityModel> position = (*j)->GetObject<MobilityModel>();
                    Vector pos = position->GetPosition();
                    
                    
                    
                    oss1 << pos.x;
                    v.push_back(oss1.str());
                    oss.clear();
                    
                    oss2 << pos.y;
                    v.push_back(oss2.str());
                    oss.clear(); 
                    
                    

                    oss3 << pos.z;
                    v.push_back(oss3.str());
                    oss.clear();
                  
                    NS_ASSERT(position);
                    Ptr<NetDevice> netDevice = object->GetDevice(0);

                    Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();
                    NS_ASSERT(loraNetDevice);
                    
                    Ptr<ClassAEndDeviceLorawanMac> mac = loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
                    NS_ASSERT(mac);

                    // Try computing the distance from each gateway and find the best one
                    Ptr<Node> bestGateway = gateways.Get(0);
                    Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

                    // Assume devices transmit at 14 dBm
                    double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);
                    

                    for (auto currentGw = gateways.Begin(); currentGw != gateways.End(); ++currentGw) {
                        // Compute the power received from the current gateway
                        Ptr<Node> curr = *currentGw;
                        Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
                        
                        
                        Vector posit = currPosition->GetPosition();
                        //cout<<curr<<" "<<posit.x<<" "<<" "<<posit.y<<" "<<posit.z<<"\n";
                        
                        double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm

                        if (currentRxPower > highestRxPower) {
                            bestGateway = curr;
                            bestGatewayPosition = curr->GetObject<MobilityModel>();
                            highestRxPower = currentRxPower;
                        }
                    }
                    //cout<<"\n";

                    oss5 << bestGateway;
                    v.push_back(oss5.str());

                    double rxPower = highestRxPower;
                    oss8 << rxPower;
                    v.push_back(oss8.str());
                    oss.clear();
                
                
                vector<Point> cluster_coord = find_nearest_6(points,k);
                cluster_coord.push_back({pos.x,pos.y});
                
                double dist = sqrt( pow(pos.x,2) + pow(pos.y,2));
                
                pair<string,string> sf = allocate_sf(dist);
                
                double angle = atan2(pos.y, pos.x) * 180.0 / M_PI;
                
                string sf_alloc;
                
                /*if (dist<= 1200) {
                a=a%2;
                if(a==0){
                cluster_centroids[sf.first].push_back(points[k]);
                sf_alloc=sf.first;
                mac->SetDataRate(5);
                sfQuantity[0] = sfQuantity[0] + 1;
                v.push_back("7");
                }
                else{
                cluster_centroids[sf.second].push_back(points[k]);
                sf_alloc=sf.second;
                mac->SetDataRate(4);
                sfQuantity[1] = sfQuantity[1] + 1;
                v.push_back("8");
                }
                a++;
                } else if (dist <= 2400) {
                b=b%2;
                if(b==0){
                cluster_centroids[sf.first].push_back(points[k]);
                sf_alloc=sf.first; 
                mac->SetDataRate(4);
                sfQuantity[1] = sfQuantity[1] + 1;
                v.push_back("8");                
                }
                else{
                cluster_centroids[sf.second].push_back(points[k]);
                sf_alloc=sf.second;
                mac->SetDataRate(3);
                sfQuantity[2] = sfQuantity[2] + 1;
                v.push_back("9"); 
                }
                b++;                   
                } else if (dist <= 3600) {
                c=c%2;
                if(c==0){
                cluster_centroids[sf.first].push_back(points[k]);
                sf_alloc=sf.first; 
                mac->SetDataRate(3);
                sfQuantity[2] = sfQuantity[2] + 1;
                v.push_back("9"); 
                }
                else{
                cluster_centroids[sf.second].push_back(points[k]);
                sf_alloc=sf.second; 
                mac->SetDataRate(2);
                sfQuantity[3] = sfQuantity[3] + 1;
                v.push_back("10");
                }
                c++;                 
                } else if (dist <= 4800) {
                d=d%2;
                if(d==0){
                cluster_centroids[sf.first].push_back(points[k]);
                sf_alloc=sf.first; 
                mac->SetDataRate(2);
                sfQuantity[3] = sfQuantity[3] + 1;
                v.push_back("10"); 
                }
                else{
                cluster_centroids[sf.second].push_back(points[k]);
                sf_alloc=sf.second; 
                mac->SetDataRate(1);
                sfQuantity[4] = sfQuantity[4] + 1;
                v.push_back("11"); 
                }
                d++;                  
                } else if (dist <= 6000) {
                e=e%2;
                if(e==0){
                cluster_centroids[sf.first].push_back(points[k]);
                sf_alloc=sf.first;  
                mac->SetDataRate(1);
                sfQuantity[4] = sfQuantity[4] + 1;
                v.push_back("11"); 
                }
                else{
                cluster_centroids[sf.second].push_back(points[k]);
                sf_alloc=sf.second; 
                mac->SetDataRate(0);
                sfQuantity[5] = sfQuantity[5] + 1;
                v.push_back("12"); 
                }
                e++;                  
                } else {
                cluster_centroids[sf.first].push_back(points[k]);
                sf_alloc=sf.first; 
                mac->SetDataRate(0);
                sfQuantity[5] = sfQuantity[5] + 1;
                v.push_back("12");                   
                }*/
                
                
                /*if(dist<=600){
                mac->SetDataRate(5);
                sfQuantity[0] = sfQuantity[0] + 1;
                v.push_back("7");
                }
                else if(dist<=750){
                mac->SetDataRate(3);
                sfQuantity[2] = sfQuantity[2] + 1;
                v.push_back("9");
                }
                else if(dist<=1050){
                a=a%2;
                if(a==0){
                cluster_centroids[sf.first].push_back(points[k]);
                sf_alloc=sf.first;
                mac->SetDataRate(5);
                sfQuantity[0] = sfQuantity[0] + 1;
                v.push_back("7");
                }
                else{
                cluster_centroids[sf.second].push_back(points[k]);
                sf_alloc=sf.second;
                mac->SetDataRate(4);
                sfQuantity[1] = sfQuantity[1] + 1;
                v.push_back("8");
                }
                a++;
                }
                else if(dist<=1350){
                mac->SetDataRate(2);
                sfQuantity[3] = sfQuantity[3] + 1;
                v.push_back("10");
                }
                else if(dist<=2250){
                b=b%2;
                if(b==0){
                cluster_centroids[sf.first].push_back(points[k]);
                sf_alloc=sf.first; 
                mac->SetDataRate(4);
                sfQuantity[1] = sfQuantity[1] + 1;
                v.push_back("8");                
                }
                else{
                cluster_centroids[sf.second].push_back(points[k]);
                sf_alloc=sf.second;
                mac->SetDataRate(3);
                sfQuantity[2] = sfQuantity[2] + 1;
                v.push_back("9"); 
                }
                b++;
                }
                else if(dist<=2550){
                mac->SetDataRate(1);
                sfQuantity[4] = sfQuantity[4] + 1;
                v.push_back("11");
                }
                else if(dist<=3450){
                c=c%2;
                if(c==0){
                cluster_centroids[sf.first].push_back(points[k]);
                sf_alloc=sf.first; 
                mac->SetDataRate(3);
                sfQuantity[2] = sfQuantity[2] + 1;
                v.push_back("9"); 
                }
                else{
                cluster_centroids[sf.second].push_back(points[k]);
                sf_alloc=sf.second; 
                mac->SetDataRate(2);
                sfQuantity[3] = sfQuantity[3] + 1;
                v.push_back("10");
                }
                c++; 
                }
                else if(dist<=3750){
                mac->SetDataRate(0);
                sfQuantity[5] = sfQuantity[5] + 1;
                v.push_back("12");
                }
                else if(dist<=4650){
                d=d%2;
                if(d==0){
                cluster_centroids[sf.first].push_back(points[k]);
                sf_alloc=sf.first; 
                mac->SetDataRate(2);
                sfQuantity[3] = sfQuantity[3] + 1;
                v.push_back("10"); 
                }
                else{
                cluster_centroids[sf.second].push_back(points[k]);
                sf_alloc=sf.second; 
                mac->SetDataRate(1);
                sfQuantity[4] = sfQuantity[4] + 1;
                v.push_back("11"); 
                }
                d++;
                }
                else if(dist<=4950){
                mac->SetDataRate(0);
                sfQuantity[5] = sfQuantity[5] + 1;
                v.push_back("12");
                }
                else if(dist<=6000){
                e=e%2;
                if(e==0){
                cluster_centroids[sf.first].push_back(points[k]);
                sf_alloc=sf.first;  
                mac->SetDataRate(1);
                sfQuantity[4] = sfQuantity[4] + 1;
                v.push_back("11"); 
                }
                else{
                cluster_centroids[sf.second].push_back(points[k]);
                sf_alloc=sf.second; 
                mac->SetDataRate(0);
                sfQuantity[5] = sfQuantity[5] + 1;
                v.push_back("12"); 
                }
                e++; 
                }
                else{
                mac->SetDataRate(0);
                sfQuantity[5] = sfQuantity[5] + 1;
                v.push_back("12");
                }*/
                
                
                /*if(dist<=1200){
                mac->SetDataRate(5);
                sfQuantity[0] = sfQuantity[0] + 1;
                v.push_back("7");
                }else if(dist<=2400){
                mac->SetDataRate(4);
                sfQuantity[1] = sfQuantity[1] + 1;
                v.push_back("8");
                }else if(dist<=3600){
                mac->SetDataRate(3);
                sfQuantity[2] = sfQuantity[2] + 1;
                v.push_back("9");
                }else if(dist<=4800){
                 mac->SetDataRate(2);
                sfQuantity[3] = sfQuantity[3] + 1;
                v.push_back("10");
                }else if(dist<=6000){
                mac->SetDataRate(1);
                sfQuantity[4] = sfQuantity[4] + 1;
                v.push_back("11");
                }else{
                mac->SetDataRate(0);
                sfQuantity[5] = sfQuantity[5] + 1;
                v.push_back("12");
                }*/

                
                
                
                
                
                
                /*if(sf_alloc=="7"){
                                        mac->SetDataRate(5);
                                        sfQuantity[0] = sfQuantity[0] + 1;
                                        v.push_back("7");
                }
                else if(sf_alloc=="8"){
                mac->SetDataRate(4);
                                        sfQuantity[1] = sfQuantity[1] + 1;
                                        v.push_back("8");
                }
                else if(sf_alloc=="9"){
                mac->SetDataRate(3);
                                        sfQuantity[2] = sfQuantity[2] + 1;
                                        v.push_back("9");
                }
                else if(sf_alloc=="10"){
                mac->SetDataRate(2);
                                        sfQuantity[3] = sfQuantity[3] + 1;
                                        v.push_back("10");
                }
                else if(sf_alloc=="11"){
                mac->SetDataRate(1);
                                        sfQuantity[4] = sfQuantity[4] + 1;
                                        v.push_back("11");
                }
                else if (sf_alloc=="12"){
                mac->SetDataRate(0);
                                        sfQuantity[5] = sfQuantity[5] + 1;
                                        v.push_back("12");
                }
                else{
                // NS_LOG_DEBUG ("Device out of range");
                                        mac->SetDataRate(0);
                                        sfQuantity[6] = sfQuantity[6] + 1;
                                        // NS_LOG_DEBUG ("sfQuantity[6] = " << sfQuantity[6]);
                                        v.push_back("12");
                }*/
                
                
                sf_alloc=allocate_sf_sector(angle,dist);
                v.push_back(sf_alloc);
                if(sf_alloc=="7"){
                mac->SetDataRate(5);
                sfQuantity[0] = sfQuantity[0] + 1;
                }
                else if(sf_alloc=="8"){
                mac->SetDataRate(4);
                sfQuantity[1] = sfQuantity[1] + 1;
                }
                else if(sf_alloc=="9"){
                mac->SetDataRate(3);
                sfQuantity[2] = sfQuantity[2] + 1;
                }
                else if(sf_alloc=="10"){
                mac->SetDataRate(2);
                sfQuantity[3] = sfQuantity[3] + 1;
                }
                else if(sf_alloc=="11"){
                mac->SetDataRate(1);
                sfQuantity[4] = sfQuantity[4] + 1;
                }
                else{
                mac->SetDataRate(0);
                sfQuantity[5] = sfQuantity[5] + 1;
                }
                
                k++;
                da.push_back(v);
                v.clear();
                }
                
                
                
                
                
                
            
            //cout<<"\n";
        
    

    return sfQuantity;
}


//EXISTING ALGORITHM code function for setting SF

/*std::vector<int>
LorawanMacHelper::SetSpreadingFactorsUp(const string& filePath, int NDevices, int radius, const string& model_name, NodeContainer endDevices,
                                                          NodeContainer gateways, Ptr<LoraChannel> channel, int i)
{
    NS_LOG_FUNCTION_NOARGS();
    
    std::vector<int> sfQuantity(7, 0);
    
    ifstream file(filePath); // Open the CSV file
    if (!file.is_open()) {
        cerr << "Error: Unable to open file: " << filePath << endl;
        return sfQuantity;
    }

    std::vector<int> sfQuantity(7, 0);
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
        NS_ASSERT(mac);

        // Try computing the distance from each gateway and find the best one
        Ptr<Node> bestGateway = gateways.Get(0);
        Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel>();

        // Assume devices transmit at 14 dBm
        double highestRxPower = channel->GetRxPower(14, position, bestGatewayPosition);

        for (auto currentGw = gateways.Begin() + 1; currentGw != gateways.End(); ++currentGw)
        {
            // Compute the power received from the current gateway
            Ptr<Node> curr = *currentGw;
            Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel>();
            double currentRxPower = channel->GetRxPower(14, position, currPosition); // dBm

            if (currentRxPower > highestRxPower)
            {
                bestGateway = curr;
                bestGatewayPosition = curr->GetObject<MobilityModel>();
                highestRxPower = currentRxPower;
            }
        }

        // NS_LOG_DEBUG ("Rx Power: " << highestRxPower);
        double rxPower = highestRxPower;

        // Get the ED sensitivity
        Ptr<EndDeviceLoraPhy> edPhy = loraNetDevice->GetPhy()->GetObject<EndDeviceLoraPhy>();
        const double* edSensitivity = edPhy->sensitivity;

        if (rxPower > *edSensitivity)
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;
        }
        else if (rxPower > *(edSensitivity + 1))
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;
        }
        else if (rxPower > *(edSensitivity + 2))
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;
        }
        else if (rxPower > *(edSensitivity + 3))
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;
        }
        else if (rxPower > *(edSensitivity + 4))
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;
        }
        else if (rxPower > *(edSensitivity + 5))
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;
        }
        else // Device is out of range. Assign SF12.
        {
            // NS_LOG_DEBUG ("Device out of range");
            mac->SetDataRate(0);
            sfQuantity[6] = sfQuantity[6] + 1;
            // NS_LOG_DEBUG ("sfQuantity[6] = " << sfQuantity[6]);
        }

        /*

        // Get the Gw sensitivity
        Ptr<NetDevice> gatewayNetDevice = bestGateway->GetDevice (0);
        Ptr<LoraNetDevice> gatewayLoraNetDevice = gatewayNetDevice->GetObject<LoraNetDevice> ();
        Ptr<GatewayLoraPhy> gatewayPhy = gatewayLoraNetDevice->GetPhy ()->GetObject<GatewayLoraPhy>
        (); const double *gwSensitivity = gatewayPhy->sensitivity;

        if(rxPower > *gwSensitivity)
          {
            mac->SetDataRate (5);
            sfQuantity[0] = sfQuantity[0] + 1;

          }
        else if (rxPower > *(gwSensitivity+1))
          {
            mac->SetDataRate (4);
            sfQuantity[1] = sfQuantity[1] + 1;

          }
        else if (rxPower > *(gwSensitivity+2))
          {
            mac->SetDataRate (3);
            sfQuantity[2] = sfQuantity[2] + 1;

          }
        else if (rxPower > *(gwSensitivity+3))
          {
            mac->SetDataRate (2);
            sfQuantity[3] = sfQuantity[3] + 1;
          }
        else if (rxPower > *(gwSensitivity+4))
          {
            mac->SetDataRate (1);
            sfQuantity[4] = sfQuantity[4] + 1;
          }
        else if (rxPower > *(gwSensitivity+5))
          {
            mac->SetDataRate (0);
            sfQuantity[5] = sfQuantity[5] + 1;

          }
        else // Device is out of range. Assign SF12.
          {
            mac->SetDataRate (0);
            sfQuantity[6] = sfQuantity[6] + 1;

          }
          */

    /*} // end loop on nodes

    return sfQuantity;

}*/

            


std::vector<int>
LorawanMacHelper::SetSpreadingFactorsGivenDistribution(NodeContainer endDevices,
                                                       NodeContainer gateways,
                                                       std::vector<double> distribution)
{
    NS_LOG_FUNCTION_NOARGS();

    std::vector<int> sfQuantity(7, 0);
    Ptr<UniformRandomVariable> uniformRV = CreateObject<UniformRandomVariable>();
    std::vector<double> cumdistr(6);
    cumdistr[0] = distribution[0];
    for (int i = 1; i < 7; ++i)
    {
        cumdistr[i] = distribution[i] + cumdistr[i - 1];
    }

    NS_LOG_DEBUG("Distribution: " << distribution[0] << " " << distribution[1] << " "
                                  << distribution[2] << " " << distribution[3] << " "
                                  << distribution[4] << " " << distribution[5]);
    NS_LOG_DEBUG("Cumulative distribution: " << cumdistr[0] << " " << cumdistr[1] << " "
                                             << cumdistr[2] << " " << cumdistr[3] << " "
                                             << cumdistr[4] << " " << cumdistr[5]);

    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<Node> object = *j;
        Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
        NS_ASSERT(position);
        Ptr<NetDevice> netDevice = object->GetDevice(0);
        Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice>();
        NS_ASSERT(loraNetDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            loraNetDevice->GetMac()->GetObject<ClassAEndDeviceLorawanMac>();
        NS_ASSERT(mac);

        double prob = uniformRV->GetValue(0, 1);

        // NS_LOG_DEBUG ("Probability: " << prob);
        if (prob < cumdistr[0])
        {
            mac->SetDataRate(5);
            sfQuantity[0] = sfQuantity[0] + 1;
        }
        else if (prob > cumdistr[0] && prob < cumdistr[1])
        {
            mac->SetDataRate(4);
            sfQuantity[1] = sfQuantity[1] + 1;
        }
        else if (prob > cumdistr[1] && prob < cumdistr[2])
        {
            mac->SetDataRate(3);
            sfQuantity[2] = sfQuantity[2] + 1;
        }
        else if (prob > cumdistr[2] && prob < cumdistr[3])
        {
            mac->SetDataRate(2);
            sfQuantity[3] = sfQuantity[3] + 1;
        }
        else if (prob > cumdistr[3] && prob < cumdistr[4])
        {
            mac->SetDataRate(1);
            sfQuantity[4] = sfQuantity[4] + 1;
        }
        else
        {
            mac->SetDataRate(0);
            sfQuantity[5] = sfQuantity[5] + 1;
        }

    } // end loop on nodes

    return sfQuantity;

} //  end function

} // namespace lorawan
} // namespace ns3
