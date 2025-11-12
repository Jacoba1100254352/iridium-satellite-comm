//
// Created by Jacob Anderson on 11/11/25.
//

#ifndef IRIDIUM_SATELLITE_COMM_PRINT_FUNCTIONS_H
#define IRIDIUM_SATELLITE_COMM_PRINT_FUNCTIONS_H

#include "../include/config.h"

// ===== SBDIX parsed fields (updated by console callback) =====
static volatile bool gSBDIXSeen = false;
static volatile int  gMOStatus = -1, gMOMSN = -1, gMTStatus = -1, gMTMSN = -1, gMTLen = -1, gMTQueued = -1;

static const char* moStatusToStr(const int code) {
  switch (code) {
    case 0:  return "MO success";
    case 1:  return "MO success, MT pending";
    case 2:  return "MO cancelled";
    case 3:  return "MO RF link lost";
    case 4:  return "MO retry limit reached";
    case 5:  return "MO SBD message too large";
    case 6:  return "MO protocol error";
    case 7:  return "MO IMEI blocked";
    case 8:  return "MO ring queue full";
    case 10: return "MO SBD option not subscribed";
    case 12: return "MO invalid input";
    case 13: return "MO radio disabled";
    case 14: return "MO ISU busy";
    case 16: return "MO network failure";
    case 32: return "MO no network service";
    default: return "MO unknown";
  }
}

static const char* mtStatusToStr(const int code) {
  switch (code) {
    case 0:  return "No MT message";
    case 1:  return "MT message received";
    case 2:  return "MT error during retrieval";
    default: return "MT unknown";
  }
}

static void printModemGlossaryOnce() {
  static bool shown = false; if (shown) return; shown = true;
  SerialMon.println();
  SerialMon.println("Glossary:");
  SerialMon.println("  AT       = 'Attention' modem command prefix (standard Hayes commands).");
  SerialMon.println("  CSQ      = Signal quality (0..5) reported by the modem.");
  SerialMon.println("  CGMR     = Firmware / revision info.");
  SerialMon.println("  SBDWB    = 'SBD Write Binary' → prepare to upload an MO payload.");
  SerialMon.println("              Flow: READY → send N bytes → modem replies '0' for checksum OK → OK.");
  SerialMon.println("  SBDIX    = 'SBD Session' → perform send/receive with network; returns 6 fields:");
  SerialMon.println("              MO-status, MOMSN, MT-status, MTMSN, MT-length, MT-queued.");
  SerialMon.println("  MSSTM    = Modem's internal tick used by a known Iridium workaround (not wall time).");
  SerialMon.println("  MO / MT  = Mobile Originated (outbound) / Mobile Terminated (inbound).");
  SerialMon.println();
}

// --- Compact printer: concise one-liner ---
static void printSBDIXCompact() {
  if (!gSBDIXSeen) return;
  SerialMon.print("SBDIX: MO="); SerialMon.print(gMOStatus); SerialMon.print(" ("); SerialMon.print(moStatusToStr(gMOStatus)); SerialMon.print(")");
  SerialMon.print(", MOMSN=");  SerialMon.print(gMOMSN);
  SerialMon.print(", MT=");     SerialMon.print(gMTStatus); SerialMon.print(" ("); SerialMon.print(mtStatusToStr(gMTStatus)); SerialMon.print(")");
  SerialMon.print(", MTMSN=");  SerialMon.print(gMTMSN);
  SerialMon.print(", MTLEN=");  SerialMon.print(gMTLen);
  SerialMon.print(", MTQ=");    SerialMon.println(gMTQueued);
}

// --- Verbose printer: only compiled/emitted when DIAGNOSTICS is true ---
#if IF_VERBOSE
static void printSBDIXVerbose() {
  if (!gSBDIXSeen) return;
  SerialMon.print("SBDIX → ");
  SerialMon.print("MO-status="); SerialMon.print(gMOStatus); SerialMon.print(" ["); SerialMon.print(moStatusToStr(gMOStatus)); SerialMon.print("]");
  SerialMon.print(", MOMSN=");    SerialMon.print(gMOMSN);
  SerialMon.print(", MT-status=");SerialMon.print(gMTStatus); SerialMon.print(" ["); SerialMon.print(mtStatusToStr(gMTStatus)); SerialMon.print("]");
  SerialMon.print(", MTMSN=");    SerialMon.print(gMTMSN);
  SerialMon.print(", MT-length=");SerialMon.print(gMTLen);
  SerialMon.print(", MT-queued=");SerialMon.println(gMTQueued);
}

static void printSBDIXLegendOnce() {
  static bool shown = false; if (shown) return; shown = true;
  SerialMon.println();
  SerialMon.println("SBDIX fields:");
  SerialMon.println("  MO = Mobile Originated (outbound) status code");
  SerialMon.println("  MOMSN = Mobile Originated Message Sequence Number");
  SerialMon.println("  MT = Mobile Terminated (inbound) status code");
  SerialMon.println("  MTQ = Mobile Terminated messages queued at gateway");
  SerialMon.println("  (Verbose adds MTMSN and MT-length)");
  SerialMon.println();
}
#endif

// ===== AT transaction pretty printer =====
// We consume lines coming from the library's console/diag callbacks and
// re-emit concise, structured messages. A tiny state machine groups SBDWB, SBDIX, MSSTM.

enum class DiagCmd { NONE, SBDWB, SBDIX, MSSTM, OTHER };
static auto  gDiagCmd     = DiagCmd::NONE;
static int   gWBExpected  = -1;
static bool  gWBReady     = false;

static void diagResetWB() { gWBExpected = -1; gWBReady = false; gDiagCmd = DiagCmd::NONE; }

static void diagPrintTX(const char* s) {
  #if IF_VERBOSE
    SerialMon.print("TX: "); SerialMon.println(s);
    return;
  #elif IF_COMPACT
    // Group transactions with a blank line
    if      (strncmp(s, "AT+SBDWB=", 9) == 0) { SerialMon.println(); SerialMon.print("AT: SBD Write Binary (bytes="); SerialMon.print(atoi(s+9)); SerialMon.println(")"); }
    else if (strncmp(s, "AT+SBDIX", 8)  == 0) { SerialMon.println(); SerialMon.println("AT: SBD Session (send/receive)"); }
    else if (strncmp(s, "AT-MSSTM", 8)  == 0) { SerialMon.println("AT: Modem tick (MSSTM)"); }
    else if (strncmp(s, "AT+CSQ", 6)    == 0) { SerialMon.println("AT: Query signal quality (CSQ)"); }
    else if (strncmp(s, "AT+CGMR", 7)   == 0) { SerialMon.println("AT: Query firmware (CGMR)"); }
  #else
  // In QUIET mode, suppress all TX content
  #endif
}

static void diagPrintRX(const char* s) {
  #if IF_VERBOSE
    SerialMon.print("RX: "); SerialMon.println(s);
    return;
  #elif IF_COMPACT
    if (s[0] == '\0') return;                               // ignore blanks
    if (strncmp(s, "AT+", 3) == 0 || strncmp(s, "AT-", 3) == 0) return; // hide raw echoes
    if (s[0] == '[') return;                                 // hide checksum/byte dump lines

    if (strcmp(s, "READY") == 0 && gDiagCmd == DiagCmd::SBDWB) {
      gWBReady = true;
      SerialMon.print("SBDWB: READY (expect "); SerialMon.print(gWBExpected); SerialMon.println(" bytes)");
    }
    else if (strcmp(s, "0") == 0 && gDiagCmd == DiagCmd::SBDWB) {
      SerialMon.println("SBDWB: checksum OK (0)");
    }
    else if (strcmp(s, "OK") == 0) {
      if (gDiagCmd == DiagCmd::SBDWB && gWBReady) {
        SerialMon.println("SBDWB: complete");
        diagResetWB();
      }
      // otherwise suppress generic OK
    }
    else if (strncmp(s, "-MSSTM:", 7) == 0) {
      // Summarize instead of printing the hex tick every time:
      SerialMon.println("MSSTM: pacing / back-off tick read\n");
    }
    else if (strncmp(s, "+CSQ:", 5) == 0) {
      SerialMon.print("CSQ: "); SerialMon.println(s + 6); // show just the number
    }
    else if (strncmp(s, "+SBDIX:", 7) == 0) {
      // parsed elsewhere; suppress here
    }
    else if (strncmp(s, "Waiting for response", 20) == 0) {
      // hide wait lines
    }
    else if (strcmp(s, "ERROR") == 0) {
      SerialMon.println("AT: ERROR");
    }
    else {
      // Print other unsoliciteds (rare)
      SerialMon.println(s);
    }
  #else
    // In QUIET mode, suppress all RX content
  #endif
}

// Route a full line coming from the console stream (">> ..." or "<< ...")
static void diagIngestConsoleLine(const char* line) {
  if (strncmp(line, ">> ", 3) == 0) {
    const char* cmd = line + 3;
    diagPrintTX(cmd);
    if      (strncmp(cmd, "AT+SBDWB=", 9) == 0) { gDiagCmd = DiagCmd::SBDWB; gWBExpected = atoi(cmd + 9); gWBReady = false; }
    else if (strncmp(cmd, "AT+SBDIX", 8)  == 0) { gDiagCmd = DiagCmd::SBDIX; }
    else if (strncmp(cmd, "AT-MSSTM", 8)  == 0) { gDiagCmd = DiagCmd::MSSTM; }
    else                                        { gDiagCmd = DiagCmd::OTHER; }
    return;
  }
  if (strncmp(line, "<< ", 3) == 0) {
    diagPrintRX(line + 3);
    return;
  }
  // Plain content (neither ">>" nor "<<"): treat as RX
  diagPrintRX(line);
}

// Feed diagnostic lines (e.g., "Waiting for response OK"). Gate on DIAG_VERBOSE.
static void diagIngestDiagLine(const char* line) {
#if IF_VERBOSE
  SerialMon.print("DBG: "); SerialMon.println(line);
#elif  IF_COMPACT
  // Plain line (neither ">>" nor "<<"): treat as RX content, but ignore pure echoes
  if (strncmp(line, "AT+", 3) == 0 || strncmp(line, "AT-", 3) == 0) return; // suppress "AT+SBDIX" etc. duplicates
  diagPrintRX(line);
#else
  // In QUIET mode, suppress all diag content
#endif
}

#endif // IRIDIUM_SATELLITE_COMM_PRINT_FUNCTIONS_H
