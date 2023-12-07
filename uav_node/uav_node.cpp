#include "uav_node.h"

using namespace std;

void uav_node::set_loc(location _loc)
{
    loc = _loc;
}
location uav_node::get_loc() const
{
    return loc;
}

void uav_node::set_bitsta(void (*fun)(std::vector<int> &_bitsta, int _id))
{
    fun(bitsta, id);
}
std::vector<int> &uav_node::get_bitsta()
{
    return bitsta;
}

void uav_node::set_id(int _id)
{
    id = _id;
}

void uav_node::set_current_sta(int sta)
{
    current_sta = sta;
}
int uav_node::get_current_sta() const
{
    return current_sta;
}

void uav_node::set_transmission_sta(int sta)
{
    transmission_sta = sta;
}
int uav_node::get_transmission_sta() const
{
    return transmission_sta;
}

void uav_node::set_scanned_sector(int sector)
{
    scanned_sector = sector;
}
int uav_node::get_scanned_sector() const
{
    return scanned_sector;
}

void uav_node::clear_receive_buffer()
{
    receive_buffer = MES_EMPTY;
}
void uav_node::receive_message(std::string mes)
{
    if (receive_buffer == MES_EMPTY)
        receive_buffer = mes;
    else
        receive_buffer = MES_ERR;
}
std::string uav_node::get_receive_buffer() const
{
    return receive_buffer;
}
