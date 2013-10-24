#include "board.h"
#include <avr/pgmspace.h>
#include "log.h"
#include "ds1339.h"
#include "qfs.h"
#include "fswrapper.h"
#include "string.h"
#include "display.h"
#ifdef HAS_BATTERY
#include "battery.h"            // do not log on battery low
#endif

static fs_inode_t logfd = 0xffff;
static uint16_t logoffset;
static char syslog[] = "Syslog.0";

void
log_init(void)
{
  logfd = fs_get_inode(&fs, syslog);
  if(logfd == 0xffff) {
    if(fs_create(&fs, syslog) != FS_OK)
      return;
    logfd = fs_get_inode(&fs, syslog);
    fs_sync(&fs);
    logoffset = 0;
  } else {
    logoffset = fs_size(&fs, logfd);
  }
}

void
log_rotate(void)
{
  char oldlog[sizeof(syslog)];
  strcpy(oldlog, syslog);

  for(int8_t i = LOG_NRFILES-2; i >= 0; i--) {
    syslog[7] = (i+0)+'0';
    oldlog[7] = (i+1)+'0';
    fs_rename(&fs, syslog, oldlog);
  }
  syslog[7] = '0';
  log_init();
}

static void
fmtdec(uint8_t d, uint8_t *out)
{
  out[0] = (d>>4) + '0';
  out[1] = (d&0xf) + '0';
}

void
Log(char *data)
{
  uint8_t now[6], fmtnow[LOG_TIMELEN+1];
  static uint8_t synced = 0;

#ifdef HAS_BATTERY
  if(battery_state < 10) { // If battery goes below 10%, sync and stop logging

    if(!synced) {
      fs_sync(&fs);
      synced = 1;
    }
    return;

  } else {
    synced = 0;
  }
#endif

  if(logfd == 0xffff)
    return;

  if(logoffset >= 1024)
    log_rotate();
  rtcget(now);

  // 0314 09:00:00
  fmtdec(now[1], fmtnow);
  fmtdec(now[2], fmtnow+ 2); fmtnow[ 4] = ' ';
  fmtdec(now[3], fmtnow+ 5); fmtnow[ 7] = ':';
  fmtdec(now[4], fmtnow+ 8); fmtnow[10] = ':';
  fmtdec(now[5], fmtnow+11);
  fmtnow[13] = ' ';

  fs_write(&fs, logfd, fmtnow, logoffset, LOG_TIMELEN+1);
  logoffset += LOG_TIMELEN+1;

  uint8_t len = strlen(data);
  data[len++] = '\n';
  fs_write(&fs, logfd, data, logoffset, len);
  logoffset += len;
}
