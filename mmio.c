#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdint.h>
#include <endian.h>
#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

volatile unsigned int *mmio;
#define REG32(m,x) ((volatile uint32_t *)((uint32_t)(m)+(uint32_t)(x)))
#define TESTSIZE (1024*1024*4)


static void setup_io(unsigned int value) {
  int fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (fd < 0) {
    printf("Unable to open /dev/mem. Run as root using sudo?\n");
    exit(-1);
  }

  void *gpio_map =
      mmap(NULL,                   // Any adddress in our space will do
           0x100,      // Map length
           PROT_READ | PROT_WRITE, // Enable reading & writting to mapped memory
           MAP_SHARED,             // Shared with other processes
           fd,                     // File to map
           value       // Offset to GPIO peripheral
      );

  close(fd);

  if (gpio_map == MAP_FAILED) {
    printf("mmap failed, errno = %d\n", errno);
    exit(-1);
  }

  mmio = ((volatile unsigned *)gpio_map);
}



volatile unsigned int test;

int main (int argc, char *argv[]) {

if (argc < 3) {
        printf("usage : mmio [base] [offset] (values in hex, without 0x)\n");
        return 1;
    }

  unsigned int value;
  unsigned int offset;
  value = strtol(argv[1], NULL, 16);
  offset = strtol(argv[2], NULL, 16);


setup_io(value);

printf("Read mmio address: %x\n",value+offset);

unsigned int a=0;
volatile unsigned int b=0;
unsigned int i = 0;

uint64_t nanosec(struct timeval t) { /* Calculate nanoseconds in a timeval structure */
  return t.tv_sec * 1000000000 + t.tv_usec * 1000;
}

/*Measure Bandwidth*/
struct timeval tval_before, tval_after, tval_result;
gettimeofday(&tval_before, NULL);
for(a=0; a<TESTSIZE; a++) {
   b= *REG32(mmio+offset, 0);
}
gettimeofday(&tval_after, NULL);
timersub(&tval_after, &tval_before, &tval_result);

uint16_t readcycle = (uint16_t)(nanosec(tval_result) / TESTSIZE);
printf("Read cycle time: %d ns\n",readcycle);
printf("Read bandwidth: %2.2fMB/sec\n\n", (1/(float)readcycle)*1000*4 );
}
