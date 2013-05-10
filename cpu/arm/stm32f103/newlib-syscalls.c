#include <debug-uart.h>
#include <sys/times.h>
#include <sys/time.h>
#include <sys/unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdio.h>
#include "models.h"
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f1/rtc.h>
#define STDOUT_USART USART2
#define STDERR_USART USART2
#define STDIN_USART USART2

#undef errno
extern int errno;
/*
 environ
 A pointer to a list of environment variables and their values.
 For a minimal environment, this empty list is adequate:
 */
char *__env[1] = { 0 };
char **environ = __env;

int _write(int file, char *ptr, int len);

void _exit(int status) {
    _write(1, "exit", 4);
    while (1) {
        ;
    }
}

int _close(int file) {
	if (file == 1 || file == 2) {
		dbg_drain();
		return 0;
	}
	errno = EBADF;
	return -1;
}

/*
 execve
 Transfer control to a new process. Minimal implementation (for a system without processes):
 */
int _execve(char *name, char **argv, char **env) {
    errno = ENOMEM;
    return -1;
}
/*
 fork
 Create a new process. Minimal implementation (for a system without processes):
 */

int _fork() {
    errno = EAGAIN;
    return -1;
}
/*
 fstat
 Status of an open file. For consistency with other minimal implementations in these examples,
 all files are regarded as character special devices.
 The `sys/stat.h' header file required is distributed in the `include' subdirectory for this C library.
 */
int _fstat(int file, struct stat *st) {
	if (file >= 0 && file <= 2) {
		st->st_mode = S_IFCHR;
		return 0;
	}
	errno = EBADF;
	return -1;
}
/*
 getpid
 Process-ID; this is sometimes used to generate strings unlikely to conflict with other processes. Minimal implementation, for a system without processes:
 */

pid_t _getpid(void) {
	return 1;
}
int
_gettimeofday(struct timeval *tv, struct timezone *tz)
{
  tv->tv_sec = rtc_get_counter_val();
  tv->tv_usec = 0;
  return 0;
}
/*
 isatty
 Query whether output stream is a terminal. For consistency with the other minimal implementations,
 */
int _isatty(int file) {
    switch (file){
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
        return 1;
    default:
        //errno = ENOTTY;
        errno = EBADF;
        return 0;
    }
}


/*
 kill
 Send a signal. Minimal implementation:
 */
int _kill(int pid, int sig) {
    errno = EINVAL;
    return (-1);
}

/*
 link
 Establish a new name for an existing file. Minimal implementation:
 */

int _link(char *old, char *new) {
    errno = EMLINK;
    return -1;
}

/*
 lseek
 Set position in a file. Minimal implementation:
 */
int _lseek(int file, int ptr, int dir) {
    return 0;
}

/*
 sbrk
 Increase program data space.
 Malloc and related functions depend on this
 */

//caddr_t _sbrk(int incr) {
//	extern char __heap_start__; /* Defined by the linker */
//	extern char __heap_end__; /* Defined by the linker */
//	static char *heap_end = &__heap_start__;
//	char *prev_heap_end;
//
//	prev_heap_end = heap_end;
//	if (heap_end + incr > &__heap_end__) {
//		printf("Heap full (requested %d, available %d)\n", incr,
//				(int) (&__heap_end__ - heap_end));
//		errno = ENOMEM;
//		return (caddr_t) -1;
//	}
//
//	heap_end += incr;
//	return (caddr_t) prev_heap_end;
//}
caddr_t _sbrk(int incr)
{
  extern char _ebss;
  static char *heap_end;
  char *prev_heap_end;

  if(heap_end == 0) {
    heap_end = &_ebss;
  }
  prev_heap_end = heap_end;

  char *stack = (char *)get_msp();

  if(heap_end + incr > stack) {
    _write(STDERR_FILENO, "Heap and stack collision", 25);
    errno = ENOMEM;
    return (caddr_t) - 1;
    //abort ();
  }

  heap_end += incr;
  return (caddr_t) prev_heap_end;

}
/*
 stat
 Status of a file (by name). Minimal implementation:
 int    _EXFUN(stat,( const char *__path, struct stat *__sbuf ));
 */

int _stat(const char *filepath, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

/*
 times
 Timing information for current process. Minimal implementation:
 */

clock_t _times(struct tms *buf) {
    return -1;
}

/*
 unlink
 Remove a file's directory entry. Minimal implementation:
 */
int _unlink(char *name) {
    errno = ENOENT;
    return -1;
}

/*
 wait
 Wait for a child process. Minimal implementation:
 */
int _wait(int *status) {
    errno = ECHILD;
    return -1;
}


int _open(const char *name, int flags, int mode) {
	errno = ENOENT;
	return -1;
}

/*
int _read(int file, char *ptr, int len) {
    uint32_t counter = len;
    if (file == 1 || file == 2) {                        // stdin from UARTx
        while(counter-- > 0) {                          // Read the character to the buffer from UART
			*ptr = usart_recv_blocking(STDIN_USART);
			ptr++;
        }
        return len;
    } else{
    	return len;
    }
}
*/
//int _read(int file, char *ptr, int len)
//{
//  char c = 0x00;
//  uint32_t counter = len;
//  switch (file) {
//  case STDIN_FILENO:
//	  while(counter-- > 0) {                          // Read the character to the buffer from UART
//		    uart_getchar(&c);
//		    *ptr++ = c;
//	  }
//    return len;
//    break;
//  default:
//    errno = EBADF;
//    return -1;
//  }
//}
/*
int _write(int file, char *ptr, int len) {
	int sent = -1;
	if (file == 1 || file == 2) {
		sent = dbg_send_bytes((const unsigned char*) ptr, len);
	}
	return sent;
}*/
//int
//_write(int file, char *ptr, int len)
//{
//  int n;
//  char c;
//
//  switch (file) {
//  case STDOUT_FILENO:          /*stdout */
//    for(n = 0; n < len; n++) {
//        c = (uint8_t) * ptr++;
//        uart_putchar(c);
//    }
//    break;
//  case STDERR_FILENO:          /* stderr */
//    for(n = 0; n < len; n++) {
//        c = (uint8_t) * ptr++;
//        uart_putchar(c);
//    }
//    break;
//  default:
//    errno = EBADF;
//    return -1;
//  }
//  return len;
//}
/*int _write(int file, char *ptr, int len)
{
	int i;

	if (file == 1) {
		for (i = 0; i < len; i++)
			usart_send_blocking(DBG_UART, ptr[i]);
		return i;
	}

	errno = EIO;
	return -1;
}*/
int fsync(int fd) {
	if (fd == 1 || fd == 2) {
		dbg_drain();
		return 0;
	}
	if (fd == 0)
		return 0;
	errno = EBADF;
	return -1;
}


void _abort() {
	while (1)
		;
}

const unsigned long bkpt_instr = 0xe1200070;
