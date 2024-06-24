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

static struct buffer {
  size_t head, tail;
  char data[PIPE_BUF];
} rx, tx;

static int events[2];
static struct pollfd fd[3];
static struct termios saved;

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

static int get(struct buffer *b, struct pollfd *in, struct pollfd *out) {
  ssize_t count = read(in->fd, b->data + b->head, sizeof b->data - b->head);

  if (count < 0)
    return errno != EAGAIN && errno != EINTR;
  b->head += count;

  if (b->head >= sizeof b->data)
    in->events &= ~POLLIN;
  if (b->head > b->tail)
    out->events |= POLLOUT;

  return count == 0;
}

static int put(struct buffer *b, struct pollfd *in, struct pollfd *out) {
  ssize_t count = write(out->fd, b->data + b->tail, b->head - b->tail);

  if (count < 0)
    return errno != EAGAIN && errno != EINTR;
  b->tail += count;

  if (b->tail >= b->head) {
    b->head = b->tail = 0;
    in->events |= POLLIN;
    out->events &= ~POLLOUT;
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

    if (fd[0].revents & (POLLIN | POLLHUP) && get(&tx, fd, fd + 1))
      restore(), errx(1, "Failed to read from /dev/tty");
    if (fd[1].revents & (POLLIN | POLLHUP) && get(&rx, fd + 1, fd))
      restore(), errx(1, "Failed to read from %s", argv[1]);

    if (fd[0].revents & POLLOUT && put(&rx, fd + 1, fd))
      restore(), errx(1, "Failed to write to /dev/tty");
    if (fd[1].revents & POLLOUT && put(&tx, fd, fd + 1))
      restore(), errx(1, "Failed to write to %s", argv[1]);
  }

  restore();
  warnx("%s closed", argv[1]);
  return 0;
}
