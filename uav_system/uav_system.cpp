#include "uav_system.h"
#include <math.h>
#include <sstream>

using namespace std;

void uav_system::init()
{
    nei_nums = 0;
    for (int i = 0; i < sim_para->N; ++i)
    {
        for (int j = i + 1; j < sim_para->N; ++j)
        {
            double ijX = uav_nodes[j].get_loc().x - uav_nodes[i].get_loc().x;
            double ijY = uav_nodes[j].get_loc().y - uav_nodes[i].get_loc().y;
            double dis = sqrt(ijX * ijX + ijY * ijY);
            if (dis > sim_para->RADIUS)
                continue;
            double cosA = ijX / dis, sinA = ijY / dis;
            double A = 180 * acos(cosA) / PI;
            if (sinA < 0)
                A = 360 - A;
            int dirK = A / alpha;
            uav_dir_nebs[i][dirK].emplace(j);
            uav_dir_nebs[j][(dirK + sim_para->K / 2) % sim_para->K].emplace(i);
            // uav_neb_dir[i][j]=dirK;
            // uav_neb_dir[j][i]=(dirK+K/2)%K;
            ++nei_nums;
        }
    }
}
void uav_system::set_node_location()
{
    unordered_set<long> uset;
    int x, y;
    int i, MAX_LOOP = 10;
    for (int n = 0; n < sim_para->N; ++n)
    {
        i = 0;
        do
        {
            x = rand() % sim_para->range_x;
            y = rand() % sim_para->range_y;
        } while (uset.find((x << 16) | y) != uset.end() && i++ < MAX_LOOP - 1);
        if (i == MAX_LOOP)
        {
            printf("cann't reset location of nodes\n");
            exit(1);
        }
        uset.emplace((x << 16) | y);
        uav_nodes[n].set_loc(location(x, y));
        uav_nodes[n].set_id(n);
    }
}

void uav_system::reset_nodes()
{
    set_node_location();
    for (auto &uav_dir : uav_dir_nebs)
    {
        for (auto &uset : uav_dir)
        {
            uset.clear();
        }
    }
    /*for(auto& uavs : uav_neb_dir){
        for(auto& dir : uavs){
            dir = -1;
        }
    }*/
    init();
}

void uav_system::set_node_bitsta(void (*fun)(std::vector<int> &bitsta, int id))
{
    for (auto &uav : uav_nodes)
    {
        uav.set_bitsta(fun);
    }
}

void uav_system::clear_all_messages()
{
    for (auto &uav : uav_nodes)
    {
        uav.clear_receive_buffer();
    }
}

std::vector<double> uav_system::sequential_scan(
    int (*work_sta)(int t, int last_transmission_sta, int last_work_sta, int transmission_flag),
    int (*transmission_sequence)(int t, std::vector<int> &bitsta, int last_transmission_sta, int current_work_sta, int transmission_flag, int reply_id),
    int (*sector_scan_sequence)(int t, int transmission_sta, int current_work_sta, int transmission_flag))
{
    //******************************************************************************************************************
    int find_nei = 0; // the number of discovered neighboring node pairs
    vector<double> out_data;

    /**
     * In the first and second handshake, record whether the current reception is correct.
     * Set the value to 1 only if the receiving node correctly receives the corresponding information; otherwise, set it to 0
     * */
    vector<int> receive_flag(sim_para->N, 0);
    vector<int> reply_node_id(sim_para->N, -1); // nodes to reply to in the second and third handshake
    unordered_set<int> uset;                    // record the discovered neighbor pairs that have mutually identified each other
    for (int t = 0; t < sim_para->MAX_SLOT; ++t)
    {
        clear_all_messages();
        for (int n = 0; n < sim_para->N; ++n)
        {
            // determine node status, transmission/reception, and sector scanning sequences
            // the order of these three functions must not be changed; design them in a specific sequence

            uav_nodes[n].set_current_sta(work_sta(t, uav_nodes[n].get_receive_sta(), uav_nodes[n].get_current_sta(), receive_flag[n]));
            uav_nodes[n].set_receive_sta(transmission_sequence(t, uav_nodes[n].get_bitsta(), uav_nodes[n].get_receive_sta(), uav_nodes[n].get_current_sta(), receive_flag[n], reply_node_id[n]));
            uav_nodes[n].set_scanned_sector(sector_scan_sequence(t, uav_nodes[n].get_receive_sta(), uav_nodes[n].get_current_sta(), receive_flag[n]));
        }
        // within this time slot, the sending node transmits information
        for (int sender = 0; sender < sim_para->N; ++sender)
        {
            if (uav_nodes[sender].get_receive_sta() == SENDING)
            {
                // sending node
                int dest_id = reply_node_id[sender];
                reply_node_id[sender] = -1;
                // packet information, source node ID + destination node ID. in a one-way handshake probing frame, the destination node ID is set to -1
                string mes = to_string(sender) + " " + to_string(dest_id);
                int sector = uav_nodes[sender].get_scanned_sector(); // sector orientation
                // traverse all nodes in that direction
                for (auto receiver : uav_dir_nebs[sender][sector])
                {
                    // nodes in reception status, aligned with the sector
                    if (uav_nodes[receiver].get_receive_sta() == RECEIVING &&
                        ((uav_nodes[receiver].get_scanned_sector() + sim_para->K / 2) % sim_para->K) == sector)
                    {
                        uav_nodes[receiver].receive_message(mes);
                    }
                }
            }
        }
        // during this time slot, the receiving node checks for received information
        for (int receiver = 0; receiver < sim_para->N; ++receiver)
        {
            if (uav_nodes[receiver].get_receive_sta() == RECEIVING)
            {
                // receiving node
                string mes = uav_nodes[receiver].get_receive_buffer();
                // the message is not empty, and there is no collision
                if (mes != MES_EMPTY && mes != MES_ERR)
                {
                    stringstream ss(mes);
                    int src_id, dest_id;
                    ss >> src_id >> dest_id;
                    int u = src_id < receiver ? (src_id * 1000 + receiver) : (receiver * 1000 + src_id);
                    // broadcast information (one-way handshake)
                    if (dest_id == -1)
                    {
                        // in a one-way handshake state, and the two nodes have not yet discovered each other
                        if (uav_nodes[receiver].get_current_sta() == 0 && uset.find(u) == uset.end())
                        {
                            reply_node_id[receiver] = src_id;
                            receive_flag[receiver] = 1;
                        }
                        else
                        {
                            receive_flag[receiver] = 0;
                        }
                    }
                    else if (dest_id == receiver) // the information must be intended for this node
                    {
                        // three-way handshake
                        if (uav_nodes[receiver].get_current_sta() == 2)
                        {
                            if (uset.find(u) == uset.end())
                            {
                                uset.emplace(u);
                                find_nei++;
                            }
                            // regardless of whether the three-way handshake is received successfully or not, no reply is required
                            receive_flag[receiver] = 0;
                        }
                        else
                        {
                            reply_node_id[receiver] = src_id;
                            receive_flag[receiver] = 1;
                        }
                    }
                    else
                    {
                        receive_flag[receiver] = 0;
                    }
                }
            }
            else
            {
                receive_flag[receiver] = 0;
            }
        }

        if (((t + 1) % sim_para->OUT_SLOT) == 0)
        {
            out_data.emplace_back(1.0 * find_nei / nei_nums);
        }
    }
    return out_data;
}
