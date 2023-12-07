#pragma once

#include "uav_node.h"
#include <unordered_set>
#include <unordered_map>

#define PI 3.1415926535

struct parameter
{
    /// @brief number of network nodes, and it is recommended for this number to be even
    int N;
    /// @brief number of sectors per node and this number must be even
    int K;
    /// @brief node communication range
    int RADIUS;
    /// @brief spatial range x of the network
    int range_x;
    /// @brief spatial range y of the network
    int range_y;

    /// @brief maximum number of time slots per simulation, i.e., the time slot at which the simulation ends
    int MAX_SLOT;
    /// @brief output time slot interval
    int OUT_SLOT;
    /// @brief simulation loop count, resetting the node's position with each iteration
    int TIME;
};

class uav_system
{

public:
    uav_system(parameter *para) : sim_para(para)
    {
        alpha = 1.0 * 360 / sim_para->K;
        uav_nodes.resize(sim_para->N);
        set_node_location();
        uav_dir_nebs.resize(sim_para->N, std::vector<std::unordered_set<int>>(sim_para->K));
        init();
        // uav_neb_dir.resize(N,std::vector<int>(N,-1));
    }

    /// @brief reset the node's position
    void reset_nodes();

    /// @brief set the node's ID sequence; different sequence scanning methods may result in different ID sequences
    /// @param fun function pointer to configure node ID sequence
    void set_node_bitsta(void (*fun)(std::vector<int> &bitsta, int id));

    /// @brief clear the message receive buffer for all nodes
    void clear_all_messages();

    /// @brief universal function for sequence scanning under three-way handshake
    /// @param work_sta function pointer for the state transition function, design required
    /// @param transmission_sequence function pointer for the send-receive sequence function, design required
    /// @param sector_scan_sequence function pointer for sector scanning sequence function, design required
    /// @return return the neighbor discovery rate array, return out_data; where out_data[k] represents the neighbor discovery rate at k * OUT_SLOT in the algorithm.
    std::vector<double> sequential_scan(
        int (*work_sta)(int t, int last_transmission_sta, int last_work_sta, int reply_id),
        int (*transmission_sequence)(int t, std::vector<int> &bitsta, int last_transmission_sta, int current_work_sta, int reply_id),
        int (*sector_scan_sequence)(int t, int transmission_sta, int current_work_sta));

    /// @brief number of neighboring node pairs in the network
    int nei_nums;

private:
    void set_node_location();
    /// @brief after resetting the node positions, reset the position relationship table among nodes
    void init();

    const parameter *sim_para;

    /// @brief sector angle size
    double alpha;

    /// @brief all nodes in the network, with a total count of N
    std::vector<uav_node> uav_nodes;
    /// @brief two-dimensional array where each element is a set. uav_dir_nebs[n][k] represents the set of all neighboring nodes in sector direction k for node n
    std::vector<std::vector<std::unordered_set<int>>> uav_dir_nebs;

    /// @brief two-dimensional array that stores the sector numbers where neighbors are located. uavNebDir[i][j] represents the sector (number) where node j is in relation to node i. A value of -1 indicates that the two nodes are not neighbors
    // std::vector<std::vector<int>> uav_neb_dir;
};
