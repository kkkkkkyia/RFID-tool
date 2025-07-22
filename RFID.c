#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>
#include <gpiod.h>
#include <poll.h>
#include <modbus/modbus.h>
#include <stdarg.h>
#include <time.h>
#define _GNU_SOURCE
#include <pthread.h>
#include <unistd.h>
#include <stdatomic.h>

/* ========= SETTINGS ========= */
#define USE_RS485             1
#define SERIAL_PORT           "/dev/ttyUSB0"
#define D0_PIN                23
#define D1_PIN                24
#define MODBUS_SLAVE_ID       206
#define MODBUS_SLAVE_D700     100
#define MODBUS_TCP_PORT       502
#define MODBUS_BUFFER_SIZE    1
#define MODBUS_REGISTER_OFFSET 0

#define STX   0x02
#define ETX   0x03
#define CR    0x0D
#define LF    0x0A
#define BUFFER_SIZE 256

/* ========= GLOBALS ========= */
static volatile uint32_t wiegand_data = 0;
static volatile int      wiegand_bits = 0;

const char *AuthorizedCardFile = "/home/pi/testing/RFIDTest/AuthorizedCard.txt";

static uint16_t modbusFlag[MODBUS_BUFFER_SIZE] = {0};
static time_t   modbusUpdateTime = 0;

static uint16_t prevState = 0;

/* ---------- authorised-UID cache ---------- */
#define MAX_UIDS 1024
static uint32_t allowedUIDs[MAX_UIDS];
static size_t   allowedCount = 0;

/* ---------- Logging Var ------------- */
static const char *StationIDFile = "/home/pi/testing/RFIDTest/config/station_id.txt";
static const char *LogFile       = "/home/pi/testing/RFIDTest/fill_reports.txt";
static char        StationID[64] = "UNKNOWN";

static int fillCounter = 0;
bool loggingStart = 0; 
bool loggingEnd = 0;
static int sessionActive = 0;

/* ---------- Logging control ---------- */
static atomic_bool authorised = ATOMIC_VAR_INIT(false);

/* ------- Thread Control -----------*/
static atomic_int running = 1;    // 0 ? exit

/* ========= UTILITIES ========= */
static void LoadAuthorizedCards(void)
{
    FILE *file = fopen(AuthorizedCardFile, "r");
    if (!file) {
        perror("Error opening access list");
        exit(EXIT_FAILURE);
    }

    allowedCount = 0;
    char buf[64];

    while (fgets(buf, sizeof buf, file) && allowedCount < MAX_UIDS) {
        if (buf[0] == '#') continue;            
        char *end;
        uint32_t id = strtoul(buf, &end, 16);   
        if (end != buf) allowedUIDs[allowedCount++] = id;
    }
    fclose(file);
    printf("[INFO] Loaded %zu authorised UIDs\n", allowedCount);
}

static int CheckAuthorizedCards(unsigned long UID, const char *unused)
{
    (void)unused;                              
    for (size_t i = 0; i < allowedCount; ++i)
        if (allowedUIDs[i] == UID) return 1;
    return 0;
}

/* ============== Log File ================ */
static void appendLog(const char *fmt, ...)
{
    FILE *f = fopen(LogFile, "a");
    if (!f) { perror("Opening log"); return; }
    va_list ap;
    va_start(ap, fmt);
    vfprintf(f, fmt, ap);
    va_end(ap);
    fprintf(f, "\n");
    fclose(f);
}

static void logHeader(unsigned long uid) {
   appendLog("################# FILL %d #################", ++fillCounter);

   time_t     t  = time(NULL);
   struct tm *tm = localtime(&t);
   char       ts[9];
   strftime(ts, sizeof(ts), "%H:%M:%S", tm);

   appendLog("Station ID: %s", StationID);
   appendLog("CARD ID: %lX",   uid);
   appendLog("Time: %s",       ts);
   appendLog("");
}


static void logFillStart(modbus_t *mb1)
{
   sessionActive = 1;
   time_t     t  = time(NULL);
   struct tm *tm = localtime(&t);
   char       ts[9];
   strftime(ts, sizeof(ts), "%H:%M:%S", tm);

   appendLog("*****fill started*****");
   appendLog("Time: %s",       ts);

   // Read flag register 58
   uint16_t flag_reg;
   int rc = modbus_read_registers(mb1, 58, 1, &flag_reg);
   if (rc != 1) {
       appendLog("Comm fill read error: %s", modbus_strerror(errno));
       appendLog("");
       return;
   }
   appendLog("Comm fill: %s", flag_reg ? "true" : "false");

   uint16_t regs[8];
   rc = modbus_read_registers(mb1, 58, 8, regs);
   if (rc != 8) {
       appendLog("Data read error: %s", modbus_strerror(errno));
       appendLog("");
       return;
   }

   union { uint32_t i; float f; } conv;

   conv.i = ((uint32_t)regs[0] << 16) | regs[1];
   appendLog("CHSS volume: %.2f L", conv.f);

   conv.i = ((uint32_t)regs[2] << 16) | regs[3];
   appendLog("CHSS Pressure: %.2f psi", conv.f);

   conv.i = ((uint32_t)regs[4] << 16) | regs[5];
   appendLog("CHSS Temperature: %.2f C", conv.f);

   conv.i = ((uint32_t)regs[6] << 16) | regs[7];
   appendLog("CHSS SOC: %.2f %%", conv.f);

   
   uint16_t regsCyl[1];
   rc = modbus_read_registers(mb1, 66, 1, regsCyl);
   appendLog("Average cylinder percentage: %.2f %%  ", rc > 0 ? regsCyl[0] / 100.0 : 0.0);

   appendLog("");
}



static void logFillEnd(modbus_t *mb1, const char *reason)
{
   time_t     t  = time(NULL);
   struct tm *tm = localtime(&t);
   char       ts[9];
   strftime(ts, sizeof(ts), "%H:%M:%S", tm);

   appendLog("*****fill ended*****");
   appendLog("Time: %s", ts);

   uint16_t regsMass[1];
   int      rc;
   rc = modbus_read_registers(mb1, 80, 1, regsMass);
   appendLog("Mass dispensed: %.2f kg ", rc > 0 ? regsMass[0] / 100.0 : 0.0);
   appendLog("Reason: %s ", reason);
   appendLog("");  

   uint16_t regs[8];
   rc = modbus_read_registers(mb1, 58, 8, regs);
   if (rc != 8) {
       appendLog("Data read error: %s", modbus_strerror(errno));
       appendLog("");
       return;
   }

   union { uint32_t i; float f; } conv;

   conv.i = ((uint32_t)regs[0] << 16) | regs[1];
   appendLog("CHSS volume: %.2f L", conv.f);

   conv.i = ((uint32_t)regs[2] << 16) | regs[3];
   appendLog("CHSS Pressure: %.2f psi", conv.f);

   conv.i = ((uint32_t)regs[4] << 16) | regs[5];
   appendLog("CHSS Temperature: %.2f C", conv.f);

   conv.i = ((uint32_t)regs[6] << 16) | regs[7];
   appendLog("CHSS SOC: %.2f %%", conv.f);

   
   uint16_t regsCyl[1];
   rc = modbus_read_registers(mb1, 66, 1, regsCyl);
   appendLog("Average cylinder percentage: %.2f %%  ", rc > 0 ? regsCyl[0] / 100.0 : 0.0);

   appendLog(""); 

   authorised = false;
}

static void handleStateTransitions(modbus_t *mb)
{
    uint16_t cur;
    if (modbus_read_registers(mb,86,1,&cur)!=1) return;

    /* detect START (1?2) */
    if (prevState!=2 && cur==2) logFillStart(mb);

    /* detect END conditions */
    if (cur!=prevState) {
        if (prevState==2 && cur!=3 && cur!=8)
            logFillEnd(mb,"Fill did not begin (no pressure or red-button)");
        else if (prevState==3 && (cur==1||cur==6))
            logFillEnd(mb,"Red-button during pressure spike");
        else if (prevState==4 && (cur==1||cur==6))
            logFillEnd(mb,"Fill ended in fill state");
        else if ((prevState==2||prevState==3||prevState==4||prevState==5) && cur==8){
            uint16_t err; char buf[80];
            if (modbus_read_registers(mb,98,1,&err)==1)
                snprintf(buf,sizeof buf,"Interlock triggered, error %u",err);
            else strcpy(buf,"Interlock triggered, error <read-fail>");
            logFillEnd(mb,buf);
        }
        else if (cur==6||cur==1)
            logFillEnd(mb,"Fill ended, undetermined reason");
    }
    prevState=cur;
}

/* ========== Thread =========== */
static void *statePollThread(void *arg)
{
    modbus_t *mb = arg;

    while (running) {
        if (atomic_load_explicit(&authorised, memory_order_acquire)) {
            handleStateTransitions(mb);
        }
        usleep(500 * 1000);      /* 500 ms */
    }
    return NULL;
}

/* ========= RS-485  ========= */
static int SetupSerial(const char *device)
{
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag  = (CLOCAL | CREAD | CS8);
    tty.c_cflag &= ~(PARENB | CSTOPB);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }
    return fd;
}

/* Forward decls */
static void VerifyAndProcess(unsigned char *data, int len, modbus_t *mb0, modbus_t *mb1);

static void ReadRS485(int fd, modbus_t *mb0, modbus_t *mb1)
{
    unsigned char buffer[BUFFER_SIZE];
    int  bufferLen = 0;

    for (;;) {
        unsigned char temp[64];
        int bytesRead = read(fd, temp, sizeof temp);

        if (bytesRead < 0) { perror("Read Error!"); break; }
        if (bytesRead == 0) { usleep(10000); continue; }

        if (bufferLen + bytesRead >= BUFFER_SIZE) {
            fprintf(stderr, "Buffer overflow!\n");
            bufferLen = 0;
            continue;
        }
        memcpy(buffer + bufferLen, temp, bytesRead);
        bufferLen += bytesRead;

        /* parse packets framed by STX / ETX */
        int i = 0;
        while (i < bufferLen) {
            if (buffer[i] != STX) { ++i; continue; }

            int j = i + 1;
            while (j < bufferLen && buffer[j] != ETX) {
                if (buffer[j] == STX) { i = j; goto next_packet; }
                ++j;
            }
            if (j >= bufferLen) break;                  

            unsigned char packet[BUFFER_SIZE];
            int packetLen = 0;
            for (int m = i + 1; m < j; ++m)
                if (buffer[m] != CR && buffer[m] != LF)
                    packet[packetLen++] = buffer[m];

            VerifyAndProcess(packet, packetLen, mb0, mb1);

            ++j;
            memmove(buffer, buffer + j, bufferLen - j);
            bufferLen -= j;
            i = 0;
        next_packet:;
        }
    }
}

static uint32_t hex_digit(char c) {
   if (c >= '0' && c <= '9') return c - '0';
   if (c >= 'A' && c <= 'F') return c - 'A' + 10;
   if (c >= 'a' && c <= 'f') return c - 'a' + 10;
   return 0;
}


/* -------- verify/process one packet -------- */
static void VerifyAndProcess(unsigned char *data, int len, modbus_t *mb0, modbus_t *mb1)
{
   uint16_t regs[1];
   printf("Hex Bytes: ");
   for (int i = 0; i < len; ++i) printf("%02X ", data[i]);
   printf("\n");

   char *asciiString = malloc(len + 1);
   memcpy(asciiString, data, len);
   asciiString[len] = '\0';
   printf("ASCII String: %s\n", asciiString);

   unsigned long UID = strtoul(asciiString, NULL, 16) >> 1;               
   printf("UID: 0x%lX (%lu)\n", UID, UID);
   logHeader(UID);

   free(asciiString);

   if (CheckAuthorizedCards(UID, AuthorizedCardFile)) {
       printf("Access Granted : updating Modbus…\n");
       modbusFlag[0]    = 1;
       modbusUpdateTime = time(NULL);

       int rc = modbus_write_registers(mb0, MODBUS_REGISTER_OFFSET, MODBUS_BUFFER_SIZE, modbusFlag);
       if (rc == -1) fprintf(stderr, "Modbus write failed: %s\n", modbus_strerror(errno));

       sleep(5);
       modbusFlag[0] = 0;
       rc = modbus_write_registers(mb0, MODBUS_REGISTER_OFFSET, MODBUS_BUFFER_SIZE, modbusFlag);
       if (rc == -1) fprintf(stderr, "Modbus write failed: %s\n", modbus_strerror(errno));
       authorised = true;
   } else {
       printf("Access Denied\n");
       return;
   }
   puts("");
   handleStateTransitions(mb1);
}

/* ========= WIEGAND ========= */
static struct gpiod_chip *chip;
static struct gpiod_line *d0_line, *d1_line;

static void onD0(int, int level, uint32_t) { if (!level) { wiegand_data <<= 1; ++wiegand_bits; } }
static void onD1(int, int level, uint32_t) { if (!level) { wiegand_data = (wiegand_data << 1) | 1; ++wiegand_bits; } }

static void setupWiegand(void)
{
    chip = gpiod_chip_open_by_name("gpiochip4");
    if (!chip) { perror("gpiod_chip_open"); exit(EXIT_FAILURE); }

    d0_line = gpiod_chip_get_line(chip, D0_PIN);
    d1_line = gpiod_chip_get_line(chip, D1_PIN);
    if (!d0_line || !d1_line) { perror("gpiod_chip_get_line"); exit(EXIT_FAILURE); }

    struct gpiod_line_request_config cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.consumer     = "wiegand";
    cfg.request_type = GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE;

    if (gpiod_line_request(d0_line, &cfg, 0) ||
        gpiod_line_request(d1_line, &cfg, 0)) {
        perror("gpiod_line_request");
        exit(EXIT_FAILURE);
    }
}

static void readWiegand(modbus_t *mb0, modbus_t *mb1)
{
   uint16_t regs[1];
   struct pollfd fds[2] = {
       { .fd = gpiod_line_event_get_fd(d0_line), .events = POLLIN },
       { .fd = gpiod_line_event_get_fd(d1_line), .events = POLLIN }
   };

   for (;;) {
       wiegand_data = 0;
       wiegand_bits = 0;

       puts("Waiting for Wiegand data (26 bits)...");
       while (wiegand_bits < 26) {
           int ret = poll(fds, 2, 1000);
           if (ret < 0) { perror("poll"); break; }
           if (ret == 0) continue;

           struct gpiod_line_event ev;
           if (fds[0].revents & POLLIN) { gpiod_line_event_read(d0_line, &ev); onD0(D0_PIN, 0, 0); }
           if (fds[1].revents & POLLIN) { gpiod_line_event_read(d1_line, &ev); onD1(D1_PIN, 0, 0); }
       }

       uint32_t raw      = wiegand_data;
       unsigned long UID = raw >> 1;

       printf("[Wiegand] Raw:0x%X  UID:0x%lX (%lu)\n", raw, UID, UID);
       logHeader(UID);

       if (CheckAuthorizedCards(UID, AuthorizedCardFile)) {
          puts("Access Granted : writing Modbus flag=1");
          modbusFlag[0]    = 1;
          modbusUpdateTime = time(NULL);
          int rc = modbus_write_registers(mb0, MODBUS_REGISTER_OFFSET, MODBUS_BUFFER_SIZE, modbusFlag);
          if (rc == -1) fprintf(stderr, "modbus write failed: %s\n", modbus_strerror(errno));
          sleep(5);
          modbusFlag[0] = 0;
          rc = modbus_write_registers(mb0, MODBUS_REGISTER_OFFSET, MODBUS_BUFFER_SIZE, modbusFlag);
          if (rc == -1) fprintf(stderr, "modbus write failed: %s\n", modbus_strerror(errno));
          authorised = true;
      } else {
          puts("Access Denied");
          continue;
      }
      puts("");
      handleStateTransitions(mb1);
   }
}

/* ========= MAIN ========= */
int main(void)
{
    puts("Starting Session…");
    LoadAuthorizedCards();

    modbus_t *mb0 = modbus_new_tcp("127.0.0.1", MODBUS_TCP_PORT);
    if (!mb0) { fputs("Failed to create Modbus context\n", stderr); return 1; }
    modbus_set_slave(mb0, MODBUS_SLAVE_ID);

    if (modbus_connect(mb0) == -1) {
        fprintf(stderr, "TCP connection failed: %s\n", modbus_strerror(errno));
        modbus_free(mb0);
        return 1;
    }
    puts("TCP connection success!");

    modbus_t *mb1 = modbus_new_tcp("127.0.0.1", MODBUS_TCP_PORT);
    if (!mb1) { fputs("Failed to create Modbus context\n", stderr); return 1; }
    modbus_set_slave(mb1, MODBUS_SLAVE_D700);

    if (modbus_connect(mb1) == -1) {
        fprintf(stderr, "TCP connection failed: %s\n", modbus_strerror(errno));
        modbus_free(mb1);
        return 1;
    }
    puts("D700 connection success!");

    /* launch polling thread */
    pthread_t tid; 
    if(pthread_create(&tid,NULL,statePollThread,mb1)!=0) {
            perror("pthread_create"); return 1;
    }

#if USE_RS485
    int fd = SetupSerial(SERIAL_PORT);
    if (fd < 0) { perror("RS485 open"); modbus_free(mb0); return 1; }
    printf("Listening on RS-485 (%s)…\n", SERIAL_PORT);
    ReadRS485(fd, mb0, mb1);
    close(fd);
#else
    setupWiegand();
    readWiegand(mb0, mb1);
#endif
    running=0; 
    pthread_join(tid,NULL); // Clean up

    modbus_close(mb0);
    modbus_close(mb1);
    modbus_free(mb0);
    modbus_free(mb1);
    return 0;
}
