//
// Created by Jacob Anderson on 11/11/25.
//

#ifndef IRIDIUM_SATELLITE_COMM_CONFIG_H
#define IRIDIUM_SATELLITE_COMM_CONFIG_H

#ifndef SerialMon
#define SerialMon Serial
#include <SerialUSB.h>
#endif

// ===== Verbosity / logging level =====
// 0 = QUIET    (no AT/MSSTM/SBDWB chatter from the pretty-printer)
// 1 = COMPACT  (friendly one-liners, grouped transactions)
// 2 = VERBOSE  (raw TX/RX lines in addition to compact summaries)
#ifndef LOG_LEVEL
#define LOG_LEVEL 1
#endif

// Backwards-compat knobs (optional)
#ifndef DIAGNOSTICS
#define DIAGNOSTICS true
#endif

// Helpers
#if (LOG_LEVEL >= 1)
#define IF_COMPACT 1
#define IF_VERBOSE 0
#define IF_QUIET 0
#elif (LOG_LEVEL >= 2)
#define IF_VERBOSE 1
#define IF_COMPACT 0
#define IF_QUIET 0
#else
#define IF_QUIET 1
#define IF_VERBOSE 0
#define IF_COMPACT 0
#endif

#endif //IRIDIUM_SATELLITE_COMM_CONFIG_H
