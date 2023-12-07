#include "uav_system.h"
#include <math.h>
#include <iostream>
#include <fstream>
#if defined(_WIN32) || defined(_WIN64)
// Windows-specific code
#include <windows.h>
#else
#include <unistd.h>
#endif

using namespace std;

ofstream of;

parameter para = {
    .N = 32,
    .K = 72,
    .RADIUS = 500,
    .range_x = 1500,
    .range_y = 1500,
    .MAX_SLOT = 1500,
    .OUT_SLOT = 36,
    .TIME = 100,
};

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
    int work_sta(int t, int last_transmission_sta, int last_work_sta, int reply_id)
    {
        if (t % 3 == 0)
            return FIRST_HANDSHAKE;
        if (t % 3 == 1)
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && reply_id != -1))
                return SECOND_HANDSHAKE;
        }
        else
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && reply_id != -1))
                return THIRD_HANDSHAKE;
        }
        return SILENT;
    }
    /// @brief SBA-D transmission and reception state transition function
    int transmission_sequence(int t, std::vector<int> &bitsta, int last_transmission_sta, int current_work_sta, int reply_id)
    {
        int bit_pos = (t / (para.K * 3)) % L;
        if (t % 3 == 0)
        {
            return bitsta[bit_pos];
        }
        else if (t % 3 == 1)
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && reply_id != -1))
            {
                return !bitsta[bit_pos];
            }
        }
        else
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && reply_id != -1))
                return bitsta[bit_pos];
        }
        return SILENT;
    }
    /// @brief SBA-D scanning sequence transition function
    int sector_scan_sequence(int t, int transmission_sta, int current_work_sta)
    {
        if (transmission_sta == SILENT)
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
    int work_sta(int t, int last_transmission_sta, int last_work_sta, int reply_id)
    {
        if ((t % (L + 2)) == 0)
        {
            return FIRST_HANDSHAKE;
        }
        if ((t % (L + 2)) < L)
        {
            if (last_work_sta == SILENT)
                return SILENT;
            if (last_transmission_sta == RECEIVING && reply_id != -1)
                return SILENT;
            else
                return FIRST_HANDSHAKE;
        }
        if ((t % (L + 2)) == L)
            return SECOND_HANDSHAKE;
        else
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && reply_id != -1))
            {
                return THIRD_HANDSHAKE;
            }
        }
        return SILENT;
    }
    /// @brief OSBA transmission and reception state transition function
    int transmission_sequence(int t, std::vector<int> &bitsta, int last_transmission_sta, int current_work_sta, int reply_id)
    {
        if (current_work_sta == SILENT)
            return SILENT;
        if (current_work_sta == FIRST_HANDSHAKE)
        {
            int bit_pos = t % (L + 2);
            return bitsta[bit_pos];
        }
        if (current_work_sta == SECOND_HANDSHAKE)
        {
            if (reply_id != -1)
                return SENDING;
            else
                return RECEIVING;
        }
        if (current_work_sta == THIRD_HANDSHAKE)
        {
            return !last_transmission_sta;
        }
        return SILENT;
    }
    /// @brief OSBA scanning sequence transition function
    int sector_scan_sequence(int t, int transmission_sta, int current_work_sta)
    {
        if (transmission_sta == SILENT)
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
    int work_sta(int t, int last_transmission_sta, int last_work_sta, int reply_id)
    {
        if (last_work_sta == THIRD_HANDSHAKE || t == 0)
            return FIRST_HANDSHAKE;
        if (last_work_sta == FIRST_HANDSHAKE)
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && reply_id != -1))
                return SECOND_HANDSHAKE;
            else
                return FIRST_HANDSHAKE;
        }
        if (last_work_sta == SECOND_HANDSHAKE)
        {
            if (last_transmission_sta == SENDING || (last_transmission_sta == RECEIVING && reply_id != -1))
                return THIRD_HANDSHAKE;
            else
                return FIRST_HANDSHAKE;
        }
        return SILENT;
    }
    /// @brief FSBA transmission and reception state transition function
    int transmission_sequence(int t, std::vector<int> &bitsta, int last_transmission_sta, int current_work_sta, int reply_id)
    {
        int bit_pos = (t - current_work_sta) % L;
        return (bitsta[bit_pos] + current_work_sta) % 2;
    }
    /// @brief FSBA scanning sequence transition function
    int sector_scan_sequence(int t, int transmission_sta, int current_work_sta)
    {
        int rt = t - current_work_sta;
        int ST = (rt / L + rt % L) % para.K;

        int et = current_work_sta == SECOND_HANDSHAKE ? (!transmission_sta) : transmission_sta;

        return (ST + et * para.K / 2) % para.K;
    }
}

string get_work_dir()
{
    char buffer[128];
#if defined(_WIN32) || defined(_WIN64)
    GetModuleFileName(NULL, buffer, sizeof(buffer) - 1);
    string full_path(buffer);
    string directory = full_path.substr(0, full_path.find_last_of("\\/"));
#else
    ssize_t count = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    buffer[count] = '\0';
    string full_path(buffer);
    string directory = full_path.substr(0, full_path.find_last_of("/"));
#endif
    cout << "current executable directory: " << directory << std::endl;
    return directory;
}

/**
 * * -n|--node             set number of network nodes
 * * -k|--sector           set number of sectors per node
 * * -r|--radius           set node communication range
 * * -x|--range_x          set spatial range x of the network
 * * -y|--range_y          set spatial range y of the network
 * * -s|--max              set maximum number of time slots per simulation
 * * -o|--out              set output time slot interval
 * * -t|--time             set simulation loop count
 */
#define REG(FUNC)                          \
    FUNC("-n", "--node", N, stoi)          \
    FUNC("-k", "--sector", K, stoi)        \
    FUNC("-r", "--radius", RADIUS, stoi)   \
    FUNC("-x", "--range_x", range_x, stoi) \
    FUNC("-y", "--range_y", range_y, stoi) \
    FUNC("-s", "--max", MAX_SLOT, stoi)    \
    FUNC("-o", "--out", OUT_SLOT, stoi)    \
    FUNC("-t", "--time", TIME, stoi)

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
    cout << "    " << #parameter << " = " << para.parameter << "   (" << option1 << "|" << option2 << ")" << endl;
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

#define _PARA(option1, option2, parameter, func) || para.parameter <= 0
#define VALUE() 0 REG(_PARA)
    if (VALUE())
    {
        cout << "all parameters need to be greater than 0\n";
        return -1;
    }
#undef VALUE
#undef _PARA

    return 0;
}

#define append_result(res1, res2)                 \
    do                                            \
    {                                             \
        if (res1.size() == 0)                     \
        {                                         \
            res1 = res2;                          \
        }                                         \
        else                                      \
        {                                         \
            for (int i = 0; i < res1.size(); ++i) \
            {                                     \
                res1[i] += res2[i];               \
            }                                     \
        }                                         \
    } while (0)

int main(int argc, char *argv[])
{
    set_parameters();
    if (check_parameter(para))
    {
        cout << "wrong parameter\n";
        return -1;
    }
    srand((unsigned)time(NULL));
    string buffer;
    buffer = get_work_dir();
    //*****************************************************************************************************************************
    SBAD::updata_L();
    OSBA::updata_L();
    FSBA::updata_L();
    uav_system *pU = new uav_system(&para);

    vector<double> result_SBAD;
    vector<double> result_OSBA;
    vector<double> result_FSBA;

    vector<double> result;

    for (int time = 0; time < para.TIME; ++time)
    {
        // cout << time << " : " << pU->nei_nums << endl;

        pU->set_node_bitsta(SBAD::set_bitsta);
        result = pU->sequential_scan(SBAD::work_sta, SBAD::transmission_sequence, SBAD::sector_scan_sequence);
        append_result(result_SBAD, result);

        pU->set_node_bitsta(OSBA::set_bitsta);
        result = pU->sequential_scan(OSBA::work_sta, OSBA::transmission_sequence, OSBA::sector_scan_sequence);
        append_result(result_OSBA, result);

        pU->set_node_bitsta(FSBA::set_bitsta);
        result = pU->sequential_scan(FSBA::work_sta, FSBA::transmission_sequence, FSBA::sector_scan_sequence);
        append_result(result_FSBA, result);

        pU->reset_nodes();
    }

    //*****************************************************************************************************************************

    string file_dir(buffer);
    string out_file = file_dir + "/result.txt";
    of.open(out_file, ios::out | ios::trunc);
    of << para.OUT_SLOT << endl;

    for (int i = 0; i < result_SBAD.size(); ++i)
    {
        result_SBAD[i] /= para.TIME;
        of << result_SBAD[i] << endl;
    }
    of << "," << endl;

    for (int i = 0; i < result_OSBA.size(); ++i)
    {
        result_OSBA[i] /= para.TIME;
        of << result_OSBA[i] << endl;
    }
    of << "," << endl;

    for (int i = 0; i < result_FSBA.size(); ++i)
    {
        result_FSBA[i] /= para.TIME;
        of << result_FSBA[i] << endl;
    }

    delete pU;

    // system("E:/lll/Documents/MATLAB/NeighborDiscoveryF/NeighborDiscovery.exe");
    // string outEx = fileDir + "/NeighborDiscovery.exe";
    // system(outEx.c_str());
}
