#ifndef _RAW_TRANSCEIVER_H
#define _RAW_TRANSCEIVER_H

#include <MyUSB/Scheduler/Scheduler.h> // Simple scheduler for task management

/* public prototypes */
void setReportState(char *in);
void radioOff(void);
void radioOnRx(void);
void restoreRadioState(void);
void addRawPulse(char *in);
void sendRawMessage(char* in);
void resetSendBuffer(char* in);
void readConfiguration(char* in);
void writeConfiguration(char* in);

extern uint8_t tx_report;

extern uint16_t credit_10ms;
#define MAX_CREDIT 500          // five second burst

TASK(ReportTask);

#endif
