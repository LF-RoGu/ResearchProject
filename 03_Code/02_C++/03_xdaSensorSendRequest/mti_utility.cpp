#include "mti_utility.h"

void get_mti_data(MTiData& latestMTiData) {
    latestMTiData = latestData;
}

void set_mti_data(const MTiData& newData) {
    latestData = newData;
}
