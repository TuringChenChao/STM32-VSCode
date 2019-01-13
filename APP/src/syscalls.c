#include <errno.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stm32f4xx.h>

#undef errno
extern int errno;

// extern int __io_putchar(int ch) __attribute__((weak));
// extern int __io_getchar(void) __attribute__((weak));

int __io_putchar(int ch)
{
    // return(ITM_SendChar(ch));
    extern UART_HandleTypeDef UartHandle;
    extern HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

    HAL_UART_Transmit(&UartHandle, (uint8_t*)&ch, 1, 10);
    return 0;
}

int __io_getchar(void)
{
    // return(ITM_ReceiveChar());
    return 0;
}

int _close(int file)
{
    return -1;
}

int _stat(char *file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    return 1;
}

int _lseek(int file, int ptr, int dir)
{
    return 0;
}

int _open(const char *name, int flags, int mode)
{
    return -1;
}

int _read(int file, char *ptr, int len)
{
    // int DataIdx;

    // for (DataIdx = 0; DataIdx < len; DataIdx++) {
    //     __io_putchar('+');
    //     *ptr++ = __io_getchar();
    //     __io_putchar('-');
    // }

    // return len;
    return 0;
}

int _write(int file, char *ptr, int len)
{
    // int DataIdx;

    // for (DataIdx = 0; DataIdx < len; DataIdx++) {
    //     __io_putchar(*ptr++);
    // }
    extern UART_HandleTypeDef UartHandle;
    extern HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    HAL_UART_Transmit(&UartHandle, (uint8_t*)ptr, len, 10);
    return len;
}

caddr_t _sbrk_r(struct _reent *r, size_t nbytes)
{
    extern char end;       /* Defined by linker */
    static char *heap_end; /* Previous end of heap or 0 if none */
    char        *prev_heap_end;
    char        *stack;

    __asm volatile("MRS %0, msp\n" : "=r"(stack));

    if (0 == heap_end) {
        heap_end = &end; /* Initialize first time round */
    }

    prev_heap_end = heap_end;

    if (stack < (prev_heap_end + nbytes)) {
     /* heap would overlap the current stack depth.
        * Future:  use sbrk() system call to make simulator grow memory beyond
        * the stack and allocate that
        */
        //errno = ENOMEM;
        return (char *) - 1;
    }
    heap_end += nbytes;

    return (caddr_t) prev_heap_end;
}

int _execve(char *name, char **argv, char **env)
{
    errno = ENOTSUP;
    return -1;
}

int _fork(void)
{
    errno = ENOTSUP;
    return -1;
}
/*
 * * getpid -- only one process, so just return 1.
 * */
#define __MYPID 1
int _getpid()
{
    return __MYPID;
}


/*
 * * kill -- go out via exit...
 * */
int _kill(int pid, int sig)
{
    errno = ENOTSUP;
    return -1;
}

int _wait(int *status)
{
    errno = ENOTSUP;
    return -1;
}

void _exit(int __status)
{
    errno = ENOTSUP;
    // return -1;
}

int _link(char *old, char *new)
{
    errno = EMLINK;
    return -1;
}

int _unlink(char *name)
{
    errno = EMLINK;
    return -1;
}
