#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

static int events[2];
static struct pollfd fd[3];
static struct termios saved;

static size_t head[2], tail[2];
static char data[2][PIPE_BUF];

static void quit(int signal) {
  if (signal)
    close(events[1]);
}

static void restore(void) {
  tcsetattr(fd[0].fd, TCSADRAIN, &saved);
  write(fd[0].fd, "\r", 1);
}

static void setup(void) {
  struct termios termios;

  if (tcgetattr(fd[0].fd, &termios) < 0)
    err(1, "tcgetattr");
  saved = termios;

  cfmakeraw(&termios);
  termios.c_lflag |= ISIG;
  termios.c_cc[VINTR] = _POSIX_VDISABLE;
  termios.c_cc[VSUSP] = _POSIX_VDISABLE;

  if (tcsetattr(fd[0].fd, TCSADRAIN, &termios) < 0)
    err(1, "tcsetattr");
}

static int get(int src, int dst) {
  ssize_t count = read(fd[src].fd, data[src] + head[src],
    sizeof data[src] - head[src]);

  if (count < 0)
    return errno != EAGAIN && errno != EINTR;
  head[src] += count;

  if (head[src] >= sizeof data[src])
    fd[src].events &= ~POLLIN;
  if (head[src] > tail[dst])
    fd[dst].events |= POLLOUT;

  return count == 0;
}

static int put(int src, int dst) {
  ssize_t count = write(fd[dst].fd, data[src] + tail[dst],
    head[src] - tail[dst]);

  if (count < 0)
    return errno != EAGAIN && errno != EINTR;
  tail[dst] += count;

  if (tail[dst] >= head[src]) {
    head[src] = tail[dst] = 0;
    fd[src].events |= POLLIN;
    fd[dst].events &= ~POLLOUT;
  }

  return 0;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s DEVICE\n", argv[0]);
    return 64;
  }

  fd[0].fd = open("/dev/tty", O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd[0].fd < 0)
    err(1, "open /dev/tty");
  if (!isatty(fd[0].fd))
    errx(1, "/dev/tty is not a tty");
  fd[0].events = POLLIN;

  fd[1].fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd[1].fd < 0)
    err(1, "open %s", argv[1]);
  if (!isatty(fd[1].fd))
    errx(1, "%s is not a tty", argv[1]);
  fd[1].events = POLLIN;

  if (pipe(events) < 0)
    err(1, "pipe");
  fd[2].fd = events[0];
  fd[2].events = POLLIN;

  signal(SIGHUP, quit);
  signal(SIGINT, quit);
  signal(SIGQUIT, quit);
  signal(SIGTERM, quit);

  setup();

  while (fd[2].revents == 0) {
    if (poll(fd, sizeof fd / sizeof *fd, -1) < 0) {
      if (errno != EAGAIN && errno != EINTR)
        restore(), err(1, "poll");
      continue;
    }

    for (int i = 0; i < 3; i++)
      if (fd[i].revents & (POLLERR | POLLNVAL))
        restore(), errx(1, "Failed to poll fd %d", fd[i].fd);

    if (fd[0].revents & (POLLIN | POLLHUP) && get(0, 1))
      restore(), errx(1, "Failed to read from /dev/tty");
    if (fd[1].revents & (POLLIN | POLLHUP) && get(1, 0))
      restore(), errx(1, "Failed to read from %s", argv[1]);

    if (fd[0].revents & POLLOUT && put(1, 0))
      restore(), errx(1, "Failed to write to /dev/tty");
    if (fd[1].revents & POLLOUT && put(0, 1))
      restore(), errx(1, "Failed to write to %s", argv[1]);
  }

  restore();
  warnx("%s closed", argv[1]);
  return 0;
}
