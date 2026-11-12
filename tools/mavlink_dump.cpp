/**
 * Reads MAVLink v2 bytes on stdin with firmware/include/mavlink_lite.h and
 * prints every verified frame as one line of JSON. Exists so
 * tools/mavlink_crosscheck.py can hand our parser frames that pymavlink
 * encoded and compare field by field: the decode direction of the cross check.
 *
 *   g++ -std=c++17 -I../firmware/include mavlink_dump.cpp -o mavlink_dump
 */

#include <cstdio>

#include "mavlink_lite.h"

using namespace mavlink;

static void print(const Frame& f) {
    std::printf("{\"msgid\":%u,\"seq\":%u,\"sys\":%u,\"comp\":%u", (unsigned)f.msgid,
                (unsigned)f.seq, (unsigned)f.system_id, (unsigned)f.component_id);
    HilSensor hs;
    RcChannelsOverride rc;
    HilActuatorControls ac;
    Heartbeat hb;
    Attitude at;
    SysStatus st;
    NamedValueFloat nv;
    if (f.decode(hs)) {
        std::printf(",\"time_usec\":%llu,\"xacc\":%.9g,\"yacc\":%.9g,\"zacc\":%.9g,"
                    "\"xgyro\":%.9g,\"ygyro\":%.9g,\"zgyro\":%.9g,\"fields_updated\":%u,\"id\":%u",
                    (unsigned long long)hs.time_usec, hs.xacc, hs.yacc, hs.zacc, hs.xgyro,
                    hs.ygyro, hs.zgyro, (unsigned)hs.fields_updated, (unsigned)hs.id);
    } else if (f.decode(rc)) {
        std::printf(",\"chan\":[%u,%u,%u,%u,%u,%u,%u,%u],\"target_system\":%u", rc.chan[0],
                    rc.chan[1], rc.chan[2], rc.chan[3], rc.chan[4], rc.chan[5], rc.chan[6],
                    rc.chan[7], (unsigned)rc.target_system);
    } else if (f.decode(ac)) {
        std::printf(",\"time_usec\":%llu,\"controls\":[%.9g,%.9g,%.9g,%.9g],\"mode\":%u",
                    (unsigned long long)ac.time_usec, ac.controls[0], ac.controls[1],
                    ac.controls[2], ac.controls[3], (unsigned)ac.mode);
    } else if (f.decode(hb)) {
        std::printf(",\"custom_mode\":%u,\"base_mode\":%u,\"system_status\":%u",
                    (unsigned)hb.custom_mode, (unsigned)hb.base_mode, (unsigned)hb.system_status);
    } else if (f.decode(at)) {
        std::printf(",\"time_boot_ms\":%u,\"roll\":%.9g,\"pitch\":%.9g", (unsigned)at.time_boot_ms,
                    at.roll, at.pitch);
    } else if (f.decode(st)) {
        std::printf(",\"voltage_battery\":%u,\"load\":%u", st.voltage_battery, st.load);
    } else if (f.decode(nv)) {
        char name[11] = {};
        for (int i = 0; i < 10; ++i) name[i] = nv.name[i];
        std::printf(",\"name\":\"%s\",\"value\":%.9g", name, nv.value);
    }
    std::printf("}\n");
}

int main() {
    Parser p;
    int c;
    while ((c = std::getchar()) != EOF) p.push(static_cast<uint8_t>(c), print);
    const ParserStats& s = p.stats();
    std::fprintf(stderr, "frames_ok=%u crc_errors=%u unknown_ids=%u seq_gaps=%u\n",
                 (unsigned)s.frames_ok, (unsigned)s.crc_errors, (unsigned)s.unknown_ids,
                 (unsigned)s.seq_gaps);
    return s.crc_errors == 0 ? 0 : 1;
}
