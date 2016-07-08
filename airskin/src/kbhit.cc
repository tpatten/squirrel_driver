#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <airskin/kbhit.h>

/**
 * example for usage:
int main(int argc, char **argv)
{
  int ch;

  kbhit_enable();

  // while no key hit do something ...
  while(!kbhit())
  {
    putchar('.');
  }

  // once a key was hit, get the actual key
  ch = getchar();

  printf("\nGot %c\n", ch);

  kbhit_disable();

  exit(EXIT_SUCCESS);
}
*/

static void changemode(bool on)
{
  // NOTE: the static structs mean this is not thread-safe!
  static struct termios oldt, newt;

  if(on)
  {
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  }
  else
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }
}

void kbhit_enable()
{
  changemode(true);
}

void kbhit_disable()
{
  changemode(false);
}

bool kbhit()
{
  struct timeval tv;
  fd_set rdfs;

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  FD_ZERO(&rdfs);
  FD_SET(STDIN_FILENO, &rdfs);

  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);
}

