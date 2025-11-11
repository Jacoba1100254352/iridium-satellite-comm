//
// Created by Jacob Anderson on 11/11/25.
//

#ifndef IRIDIUM_SATELLITE_COMM_PRINT_FUNCTIONS_H
#define IRIDIUM_SATELLITE_COMM_PRINT_FUNCTIONS_H

#ifndef DIAGNOSTICS
#define DIAGNOSTICS true
#endif

#ifndef SerialMon
#define SerialMon Serial
#include <SerialUSB.h>
#endif

// ---- Parsed SBDIX fields (updated by the console callback) ----
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

// --- Compact printer: always permitted; concise & legible ---
static void printSBDIXCompact() {
  if (!gSBDIXSeen) return;
  SerialMon.print("SBDIX: MO="); SerialMon.print(gMOStatus); SerialMon.print(" ("); SerialMon.print(moStatusToStr(gMOStatus)); SerialMon.print(")");
  SerialMon.print(", MOMSN=");    SerialMon.print(gMOMSN);
  SerialMon.print(", MT=");       SerialMon.print(gMTStatus); SerialMon.print(" ("); SerialMon.print(mtStatusToStr(gMTStatus)); SerialMon.print(")");
  SerialMon.print(", MTQ=");      SerialMon.println(gMTQueued);
}

// --- Verbose printer: only compiled/emitted when DIAGNOSTICS is true ---
#if DIAGNOSTICS
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

// Print a one-time legend so operators know what the fields mean.
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
#endif // DIAGNOSTICS

#endif // IRIDIUM_SATELLITE_COMM_PRINT_FUNCTIONS_H
