#include "uav_system.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <unistd.h> //linux
// #include <direct.h> //windows
using namespace std;

const int N = 48;
const int K = 72;
const int MAX_X = 1500;
const int MAX_Y = 1500;

// simulation loop count, resetting the node's position with each iteration
#define TIME 100

/// @brief SBA-D
namespace SBAD
{
    const int L = ceil(1.0 * log2(N));
    /// @brief  SBA-D ID sequence
    void set_bitsta(std::vector<int> &bitsta, int id)
    {
        bitsta.resize(L, 0);
        int num;
        int tar = pow(2, L) - N;
        if (id < N / 2)
        {
            num = id;
        }
        else
        {
            num = id + tar;
        }
        for (int b = 0; b < L; ++b)
        {
            bitsta[b] = (num >> (L - b - 1)) & 0x01;
        }
    }
    /// @brief SBA-D state transition function
    int work_sta(int t, int last_transmission_sta, int last_work_sta, int transmission_flag)
    {
        if (t % 3 == 0)
            return 0;
        if (t % 3 == 1)
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && transmission_flag == 1))
                return 1;
        }
        else
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && transmission_flag == 1))
                return 2;
        }
        return -1;
    }
    /// @brief SBA-D transmission and reception state transition function
    int transmission_sequence(int t, std::vector<int> &bitsta, int last_transmission_sta, int current_work_sta, int transmission_flag, int reply_id)
    {
        int bit_pos = (t / (K * 3)) % L;
        if (t % 3 == 0)
        {
            return bitsta[bit_pos];
        }
        else if (t % 3 == 1)
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && transmission_flag == 1))
            {
                return !bitsta[bit_pos];
            }
        }
        else
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && transmission_flag == 1))
                return bitsta[bit_pos];
        }
        return SILENT;
    }
    /// @brief SBA-D scanning sequence transition function
    int sector_scan_sequence(int t, int transmission_sta, int current_work_sta, int transmission_flag)
    {
        if (transmission_sta == 2)
            return -1;
        return ((t / 3) % K + ((t % 3 == 1) ? (!transmission_sta) : (transmission_sta)) * K / 2) % K;
    }
}

/// @brief OSBA
namespace OSBA
{
    const int L = ceil(1.0 * log2(N));
    /// @brief OSBA ID sequence
    void set_bitsta(std::vector<int> &bitsta, int id)
    {
        bitsta.resize(L, 0);
        int num;
        int tar = pow(2, L) - N;
        if (id < N / 2)
        {
            num = id;
        }
        else
        {
            num = id + tar;
        }
        for (int b = 0; b < L; ++b)
        {
            bitsta[b] = (num >> (L - b - 1)) & 0x01;
        }
    }
    /// @brief OSBA state transition function
    int work_sta(int t, int last_transmission_sta, int last_work_sta, int transmission_flag)
    {
        if ((t % (L + 2)) == 0)
        {
            return 0;
        }
        if ((t % (L + 2)) < L)
        {
            if (last_work_sta == -1)
                return -1;
            if (last_transmission_sta == RECEIVING && transmission_flag == 1)
                return -1;
            else
                return 0;
        }
        if ((t % (L + 2)) == L)
            return 1;
        else
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && transmission_flag == 1))
            {
                return 2;
            }
        }
        return -1;
    }
    /// @brief OSBA transmission and reception state transition function
    int transmission_sequence(int t, std::vector<int> &bitsta, int last_transmission_sta, int current_work_sta, int transmission_flag, int reply_id)
    {
        if (current_work_sta == -1)
            return SILENT;
        if (current_work_sta == 0)
        {
            int bit_pos = t % (L + 2);
            return bitsta[bit_pos];
        }
        if (current_work_sta == 1)
        {
            if (reply_id != -1)
                return SENDING;
            else
                return RECEIVING;
        }
        if (current_work_sta == 2)
        {
            return !last_transmission_sta;
        }
        return SILENT;
    }
    /// @brief OSBA scanning sequence transition function
    int sector_scan_sequence(int t, int transmission_sta, int current_work_sta, int transmission_flag)
    {
        if (transmission_sta == 2)
            return -1;
        int ST = (t / (L + 2)) % K;

        return (ST + (1 + transmission_sta + current_work_sta) * K / 2) % K;
    }

}

/// @brief incremental sequence scanning
namespace FSBA
{
    const int LF = ceil(1.0 * log2(N));
    const int L = LF % 2 == 0 ? LF + 1 : LF;
    /// @brief FSBA ID sequence
    void set_bitsta(std::vector<int> &bitsta, int id)
    {
        bitsta.resize(L, 0);
        int num;
        int tar = pow(2, L) - N;
        if (id < N / 2)
        {
            num = id;
        }
        else
        {
            num = id + tar;
        }
        for (int b = 0; b < L; ++b)
        {
            bitsta[b] = (num >> (L - b - 1)) & 0x01;
        }
    }
    /// @brief FSBA state transition function
    int work_sta(int t, int last_transmission_sta, int last_work_sta, int transmission_flag)
    {
        if (last_work_sta == 2 || t == 0)
            return 0;
        if (last_work_sta == 0)
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && transmission_flag == 1))
                return 1;
            else
                return 0;
        }
        if (last_work_sta == 1)
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && transmission_flag == 1))
                return 2;
            else
                return 0;
        }
        return -1;
    }
    /// @brief FSBA transmission and reception state transition function
    int transmission_sequence(int t, std::vector<int> &bitsta, int last_transmission_sta, int current_work_sta, int transmission_flag, int reply_id)
    {
        int bit_pos = (t - current_work_sta) % L;
        return (bitsta[bit_pos] + current_work_sta) % 2;
    }
    /// @brief FSBA scanning sequence transition function
    int sector_scan_sequence(int t, int transmission_sta, int current_work_sta, int transmission_flag)
    {
        int rt = t - current_work_sta;
        int ST = (rt / L + rt % L) % K;

        int et = current_work_sta == 1 ? (!transmission_sta) : transmission_sta;

        return (ST + et * K / 2) % K;
    }
}

ofstream of;

int main()
{
    srand((unsigned)time(NULL));
    char *buffer;
    if ((buffer = getcwd(NULL, 0)) == NULL)
    {
        perror("getcwd error");
    }
    else
    {
        cout << buffer << endl;
    }
    //*****************************************************************************************************************************
    uav_system *pU = new uav_system(N, K, MAX_X, MAX_Y);

    vector<double> result_SBAD;
    vector<double> result_OSBA;
    vector<double> result_FSBA;

    for (int time = 0; time < TIME; ++time)
    {
        cout << time << " : " << pU->nei_nums << endl;

        pU->set_node_bitsta(SBAD::set_bitsta);
        vector<double> result = pU->sequential_scan(SBAD::work_sta, SBAD::transmission_sequence, SBAD::sector_scan_sequence);
        if (result_SBAD.size() == 0)
        {
            result_SBAD = result;
        }
        else
        {
            for (int i = 0; i < result_SBAD.size(); ++i)
            {
                result_SBAD[i] += result[i];
            }
        }

        pU->set_node_bitsta(OSBA::set_bitsta);
        result = pU->sequential_scan(OSBA::work_sta, OSBA::transmission_sequence, OSBA::sector_scan_sequence);
        if (result_OSBA.size() == 0)
        {
            result_OSBA = result;
        }
        else
        {
            for (int i = 0; i < result_OSBA.size(); ++i)
            {
                result_OSBA[i] += result[i];
            }
        }

        pU->set_node_bitsta(FSBA::set_bitsta);
        result = pU->sequential_scan(FSBA::work_sta, FSBA::transmission_sequence, FSBA::sector_scan_sequence);
        if (result_FSBA.size() == 0)
        {
            result_FSBA = result;
        }
        else
        {
            for (int i = 0; i < result_FSBA.size(); ++i)
            {
                result_FSBA[i] += result[i];
            }
        }

        pU->reset_nodes();
    }

    //*****************************************************************************************************************************

    string fileDir(buffer);
    string outFile = fileDir + "/result.txt";
    of.open(outFile, ios::out | ios::trunc);
    of << OUT_SLOT << endl;

    for (int i = 0; i < result_SBAD.size(); ++i)
    {
        result_SBAD[i] /= TIME;
        of << result_SBAD[i] << endl;
    }
    of << "," << endl;

    for (int i = 0; i < result_OSBA.size(); ++i)
    {
        result_OSBA[i] /= TIME;
        of << result_OSBA[i] << endl;
    }
    of << "," << endl;

    for (int i = 0; i < result_FSBA.size(); ++i)
    {
        result_FSBA[i] /= TIME;
        of << result_FSBA[i] << endl;
    }
    of << "," << endl;

    delete pU;

    // system("E:/lll/Documents/MATLAB/NeighborDiscoveryF/NeighborDiscovery.exe");
    // string outEx = fileDir + "/NeighborDiscovery.exe";
    // system(outEx.c_str());
}
