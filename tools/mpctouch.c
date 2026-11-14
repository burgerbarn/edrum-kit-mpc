// mpctouch: inject taps/swipes into the MPC touchscreen (evdev write). Coordinates are landscape
// screen pixels (1280x800) as seen on the display; mapped to the panel's native axes.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <time.h>
static int fd; static int tid;
static struct input_absinfo ax, ay;
static void ev(int t, int c, int v) { struct input_event e; memset(&e, 0, sizeof e); e.type = t; e.code = c; e.value = v; write(fd, &e, sizeof e); }
static void syn(void) { ev(EV_SYN, SYN_REPORT, 0); }
static int mode = 0;   // 0: native = rotated (nx = sy, ny = 1279 - sx), 1: native = same as screen
static void map(int sx, int sy, int* nx, int* ny) {
  double fx, fy;
  if (mode == 0) { fx = sy / 799.0; fy = (1279 - sx) / 1279.0; }
  else if (mode == 1) { fx = sx / 1279.0; fy = sy / 799.0; }
  else if (mode == 2) { fx = (799 - sy) / 799.0; fy = sx / 1279.0; }
  else { fx = (1279 - sx) / 1279.0; fy = (799 - sy) / 799.0; }
  *nx = ax.minimum + (int)(fx * (ax.maximum - ax.minimum)); *ny = ay.minimum + (int)(fy * (ay.maximum - ay.minimum));
}
static void down(int sx, int sy) { int x, y; map(sx, sy, &x, &y); ev(EV_ABS, ABS_MT_SLOT, 0); ev(EV_ABS, ABS_MT_TRACKING_ID, tid++); ev(EV_ABS, ABS_MT_POSITION_X, x); ev(EV_ABS, ABS_MT_POSITION_Y, y); ev(EV_KEY, BTN_TOUCH, 1); ev(EV_ABS, ABS_X, x); ev(EV_ABS, ABS_Y, y); syn(); }
static void move(int sx, int sy) { int x, y; map(sx, sy, &x, &y); ev(EV_ABS, ABS_MT_POSITION_X, x); ev(EV_ABS, ABS_MT_POSITION_Y, y); ev(EV_ABS, ABS_X, x); ev(EV_ABS, ABS_Y, y); syn(); }
static void up(void) { ev(EV_ABS, ABS_MT_TRACKING_ID, -1); ev(EV_KEY, BTN_TOUCH, 0); syn(); }
static void ms(int m) { struct timespec t = { m / 1000, (m % 1000) * 1000000L }; nanosleep(&t, 0); }
int main(int argc, char** argv) {
  if (argc < 2) { fprintf(stderr, "usage: mpctouch info | [-m N] tap X Y | swipe X1 Y1 X2 Y2 | dtap X Y\n"); return 1; }
  tid = (int)(time(0) % 30000) + 1; fd = -1;
  for (int n = 0; n < 16 && fd < 0; ++n) {   // find the touchscreen by name (event number changes between boots)
    char path[32], name[128] = {0}; snprintf(path, sizeof path, "/dev/input/event%d", n);
    int f = open(path, O_RDWR); if (f < 0) continue;
    ioctl(f, EVIOCGNAME(sizeof name - 1), name);
    if (strstr(name, "Touchscreen")) fd = f; else close(f);
  } if (fd < 0) { perror("open"); return 1; }
  ioctl(fd, EVIOCGABS(ABS_MT_POSITION_X), &ax); ioctl(fd, EVIOCGABS(ABS_MT_POSITION_Y), &ay);
  int a = 1;
  if (!strcmp(argv[a], "-m")) { mode = atoi(argv[a + 1]); a += 2; }
  if (!strcmp(argv[a], "info")) { printf("x %d..%d  y %d..%d\n", ax.minimum, ax.maximum, ay.minimum, ay.maximum); return 0; }
  if (!strcmp(argv[a], "tap") || !strcmp(argv[a], "dtap")) { int x = atoi(argv[a + 1]), y = atoi(argv[a + 2]); int n = !strcmp(argv[a], "dtap") ? 2 : 1;
    for (int i = 0; i < n; ++i) { down(x, y); ms(60); up(); ms(90); } return 0; }
  if (!strcmp(argv[a], "swipe")) { int x1 = atoi(argv[a+1]), y1 = atoi(argv[a+2]), x2 = atoi(argv[a+3]), y2 = atoi(argv[a+4]);
    down(x1, y1); for (int i = 1; i <= 15; ++i) { ms(20); move(x1 + (x2 - x1) * i / 15, y1 + (y2 - y1) * i / 15); } ms(40); up(); return 0; }
  return 1;
}
