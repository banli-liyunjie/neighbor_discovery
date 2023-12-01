#include "uav_system.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <unistd.h> //linux
// #include <direct.h> //windows
using namespace std;

parameter para = {
    .N = 32,
    .K = 72,
    .RADIUS = 500,
    .range_x = 1500,
    .range_y = 1500,
    .MAX_SLOT = 1500,
    .OUT_SLOT = 36,
};

// simulation loop count, resetting the node's position with each iteration
#define TIME 100

/// @brief SBA-D
namespace SBAD
{
    int L;
    void updata_L() { L = ceil(1.0 * log2(para.N)); }
    /// @brief  SBA-D ID sequence
    void set_bitsta(std::vector<int> &bitsta, int id)
    {
        bitsta.resize(L, 0);
        int num;
        int tar = pow(2, L) - para.N;
        if (id < para.N / 2)
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
        int bit_pos = (t / (para.K * 3)) % L;
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
        return ((t / 3) % para.K + ((t % 3 == 1) ? (!transmission_sta) : (transmission_sta)) * para.K / 2) % para.K;
    }
}

/// @brief OSBA
namespace OSBA
{
    int L;
    void updata_L() { L = ceil(1.0 * log2(para.N)); }
    /// @brief OSBA ID sequence
    void set_bitsta(std::vector<int> &bitsta, int id)
    {
        bitsta.resize(L, 0);
        int num;
        int tar = pow(2, L) - para.N;
        if (id < para.N / 2)
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
        int ST = (t / (L + 2)) % para.K;

        return (ST + (1 + transmission_sta + current_work_sta) * para.K / 2) % para.K;
    }

}

/// @brief incremental sequence scanning
namespace FSBA
{
    int L;
    void updata_L()
    {
        int LF = ceil(1.0 * log2(para.N));
        L = LF % 2 == 0 ? LF + 1 : LF;
    }
    /// @brief FSBA ID sequence
    void set_bitsta(std::vector<int> &bitsta, int id)
    {
        bitsta.resize(L, 0);
        int num;
        int tar = pow(2, L) - para.N;
        if (id < para.N / 2)
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
        int ST = (rt / L + rt % L) % para.K;

        int et = current_work_sta == 1 ? (!transmission_sta) : transmission_sta;

        return (ST + et * para.K / 2) % para.K;
    }
}

ofstream of;

/**
 * * -n|--node             set number of network nodes
 * * -k|--sector           set number of sectors per node
 * * -r|--radius           set node communication range
 * * -x|--range_x          set spatial range x of the network
 * * -y|--range_y          set spatial range y of the network
 * * -s|--max              set maximum number of time slots per simulation
 * * -o|--out              set output time slot interval
 */
#define REG(FUNC)                          \
    FUNC("-n", "--node", N, stoi)          \
    FUNC("-k", "--sector", K, stoi)        \
    FUNC("-r", "--radius", RADIUS, stoi)   \
    FUNC("-x", "--range_x", range_x, stoi) \
    FUNC("-y", "--range_y", range_y, stoi) \
    FUNC("-s", "--max", MAX_SLOT, stoi)    \
    FUNC("-o", "--out", OUT_SLOT, stoi)

#define SET_PARAMETER(option1, option2, parameter, func)                            \
    if ((string(argv[i]) == option1 || string(argv[i]) == option2) && i + 1 < argc) \
    {                                                                               \
        para.parameter = func(string(argv[(++i)++]));                               \
        continue;                                                                   \
    }
#define set_parameters()                                      \
    do                                                        \
    {                                                         \
        for (int i = 1; i < argc;)                            \
        {                                                     \
            REG(SET_PARAMETER)                                \
            cout << "unknown option : \"" << argv[i]          \
                 << "\" or no value for this option" << endl; \
            return -1;                                        \
        }                                                     \
    } while (0)

#define PRINT_PARAMETER(option1, option2, parameter, func) \
    cout << "    " << #parameter << " = " << para.parameter << endl;
#define print_parameters()                     \
    do                                         \
    {                                          \
        cout << "simulation parameters : {\n"; \
        REG(PRINT_PARAMETER)                   \
        cout << "}\n";                         \
    } while (0)

int check_parameter(parameter &para)
{
    print_parameters();
    if (para.K % 2 == 1)
    {
        cout << "number of sectors (K) must be even\n";
        return 1;
    }
    if (para.N % 2 == 1)
    {
        cout << "it is recommended for this number (N) to be even\n";
    }
    if (para.N <= 0 || para.K <= 0 || para.RADIUS <= 0 || para.range_x <= 0 || para.range_y <= 0 || para.MAX_SLOT <= 0 || para.OUT_SLOT <= 0)
    {
        cout << "all parameters need to be greater than 0\n";
        return -1;
    }

    return 0;
}

int main(int argc, char *argv[])
{
    set_parameters();
    if (check_parameter(para))
    {
        cout << "wrong parameter\n";
        return -1;
    }
    srand((unsigned)time(NULL));
    char *buffer;
    if ((buffer = getcwd(NULL, 0)) == NULL)
    {
        perror("getcwd error");
        return -1;
    }
    else
    {
        cout << buffer << endl;
    }
    //*****************************************************************************************************************************
    SBAD::updata_L();
    OSBA::updata_L();
    FSBA::updata_L();
    uav_system *pU = new uav_system(&para);

    vector<double> result_SBAD;
    vector<double> result_OSBA;
    vector<double> result_FSBA;

    for (int time = 0; time < TIME; ++time)
    {
        // cout << time << " : " << pU->nei_nums << endl;

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

    string file_dir(buffer);
    string out_file = file_dir + "/result.txt";
    of.open(out_file, ios::out | ios::trunc);
    of << para.OUT_SLOT << endl;

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

    delete pU;

    // system("E:/lll/Documents/MATLAB/NeighborDiscoveryF/NeighborDiscovery.exe");
    // string outEx = fileDir + "/NeighborDiscovery.exe";
    // system(outEx.c_str());
}
