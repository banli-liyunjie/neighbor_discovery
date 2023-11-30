#pragma once

#include <string>
#include <vector>

#define RECEIVING 0
#define SENDING 1
#define SILENT 2

#define MES_EMPTY ""
#define MES_ERR "N"

struct location
{
    location() {}
    location(double _x, double _y) : x(_x), y(_y) {}

    double x;
    double y;
};

class uav_node
{

public:
    uav_node() : current_sta(0), receive_buffer(MES_EMPTY) {}
    void set_loc(location _loc);
    location get_loc() const;

    /// @brief set node ID sequence
    /// @param fun function pointer for custom node ID sequence
    void set_bitsta(void (*fun)(std::vector<int> &bitsta, int id));
    std::vector<int> &get_bitsta();

    void set_id(int id);
    void set_current_sta(int sta);
    int get_current_sta() const;
    void set_receive_sta(int sta);
    int get_receive_sta() const;
    void set_scanned_sector(int sector);
    int get_scanned_sector() const;

    /// @brief clear the message receive buffer
    void clear_receive_buffer();

    /// @brief filling the message receive buffer means the node has received a message
    /// @param mes message
    void receive_message(std::string mes);

    /// @brief return the current message buffer of the node
    /// @return
    std::string get_receive_buffer() const;

private:
    /// @brief node position
    location loc;

    /// @brief node ID number
    int id;

    /// @brief ID sequence array
    std::vector<int> bitsta;

    /// @brief current work status of the node
    int current_sta;
    /// @brief node transmission and reception status, 0: receiving, 1: sending, 2: silent
    int receive_sta;
    /// @brief scanning sector, -1: inactive, others: corresponding sector number
    int scanned_sector;

    /// @brief message receive buffer
    std::string receive_buffer;
};
