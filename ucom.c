#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

static int events[2], fd[3], limit;
static struct termios saved[2];
static fd_set rfd, wfd;

static size_t head[2], tail[2];
static char data[2][PIPE_BUF];

static void quit(int signal) {
  if (signal)
    close(events[1]);
}

static void restore(void) {
  for (int i = 0; i < 2; i++)
    tcsetattr(fd[i], TCSADRAIN, saved + i);
  write(fd[0], "\r", 1);
}

static void setup(void) {
  for (int i = 0; i < 2; i++)
    if (tcgetattr(fd[i], saved + i) < 0)
      err(1, "tcgetattr");

  for (int i = 0; i < 2; i++) {
    struct termios termios = saved[i];

    cfmakeraw(&termios);
    termios.c_cc[VMIN] = 1;
    termios.c_cc[VTIME] = 0;
    termios.c_cflag |= CLOCAL;

    if (i == 0) {
      termios.c_lflag |= ISIG;
      termios.c_cc[VINTR] = _POSIX_VDISABLE;
      termios.c_cc[VSUSP] = _POSIX_VDISABLE;
    }

    if (tcsetattr(fd[i], TCSAFLUSH, &termios) < 0)
      restore(), err(1, "tcsetattr");
  }
}

static int get(int src, int dst) {
  ssize_t count = read(fd[src], data[src] + head[src],
    sizeof data[src] - head[src]);

  if (count < 0)
    return errno != EAGAIN && errno != EINTR;
  head[src] += count;

  return count == 0;
}

static int put(int src, int dst) {
  ssize_t count = write(fd[dst], data[src] + tail[dst],
    head[src] - tail[dst]);

  if (count < 0)
    return errno != EAGAIN && errno != EINTR;
  tail[dst] += count;

  if (tail[dst] >= head[src])
    head[src] = tail[dst] = 0;

  return 0;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s DEVICE\n", argv[0]);
    return 64;
  }

  fd[0] = open("/dev/tty", O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd[0] < 0)
    err(1, "open /dev/tty");
  if (!isatty(fd[0]))
    errx(1, "/dev/tty is not a tty");

  fd[1] = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd[1] < 0)
    err(1, "open %s", argv[1]);
  if (!isatty(fd[1]))
    errx(1, "%s is not a tty", argv[1]);

  if (pipe(events) < 0)
    err(1, "pipe");
  fd[2] = events[0];

  limit = fd[0] < fd[1] ? fd[1] : fd[0];
  limit = limit < fd[2] ? fd[2] : limit;
  if (limit >= FD_SETSIZE)
    errx(1, "select() is limited to %u file descriptors", FD_SETSIZE);

  signal(SIGHUP, quit);
  signal(SIGINT, quit);
  signal(SIGQUIT, quit);
  signal(SIGTERM, quit);

  setup();

  do {
    FD_ZERO(&rfd);
    FD_ZERO(&wfd);
    FD_SET(fd[2], &rfd);

    for (int i = 0; i < 2; i++) {
      if (head[i] < sizeof data[i])
        FD_SET(fd[i], &rfd);
      if (tail[i] < head[1 - i])
        FD_SET(fd[i], &wfd);
    }

    if (select(limit + 1, &rfd, &wfd, NULL, NULL) < 0) {
      if (errno != EAGAIN && errno != EINTR)
        restore(), err(1, "select");
      continue;
    }

    if (FD_ISSET(fd[0], &rfd) && get(0, 1))
      restore(), errx(1, "Failed to read from /dev/tty");
    if (FD_ISSET(fd[1], &rfd) && get(1, 0))
      restore(), errx(1, "Failed to read from %s", argv[1]);

    if (FD_ISSET(fd[0], &wfd) && put(1, 0))
      restore(), errx(1, "Failed to write to /dev/tty");
    if (FD_ISSET(fd[1], &wfd) && put(0, 1))
      restore(), errx(1, "Failed to write to %s", argv[1]);
  } while (!FD_ISSET(fd[2], &rfd));

  restore();
  warnx("%s closed", argv[1]);
  return 0;
}
