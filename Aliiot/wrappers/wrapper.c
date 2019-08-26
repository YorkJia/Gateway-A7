/**
 * NOTE:
 *
 * HAL_TCP_xxx API reference implementation: wrappers/os/ubuntu/HAL_TCP_linux.c
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <memory.h>

#include <pthread.h>
#include <unistd.h>     //unix std
#include <sys/prctl.h>
#include <sys/time.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <net/if.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/reboot.h>
#include <time.h>
#include <signal.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/tcp.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>

#include <infra_config.h>
#include "infra_types.h"
#include "infra_defs.h"
#include "wrappers_defs.h"


char _product_key[IOTX_PRODUCT_KEY_LEN + 1]       = "a1RIsMLz2BJ";
char _product_secret[IOTX_PRODUCT_SECRET_LEN + 1] = "fSAF0hle6xL0oRWd";
char _device_name[IOTX_DEVICE_NAME_LEN + 1]       = "example1";
char _device_secret[IOTX_DEVICE_SECRET_LEN + 1]   = "RDXf67itLqZCwdMCRrw0N5FHbv5D7jrE";

/******************************************************/
/* uart config */
#define AT_UART_PORT 1
#define AT_UART_LINUX_DEV    "/dev/ttymxc1"
#define AT_UART_BAUDRATE     115200
#define AT_UART_DATA_WIDTH   DATA_WIDTH_8BIT
#define AT_UART_PARITY       NO_PARITY
#define AT_UART_STOP_BITS    STOP_BITS_1
#define AT_UART_FLOW_CONTROL FLOW_CONTROL_DISABLED
#define AT_UART_MODE         MODE_TX_RX
#define AT_UART_TIMEOUT_MS   1000



/*
 * UART data width
 */
typedef enum {
    DATA_WIDTH_5BIT,
    DATA_WIDTH_6BIT,
    DATA_WIDTH_7BIT,
    DATA_WIDTH_8BIT,
    DATA_WIDTH_9BIT
} hal_uart_data_width_t;

/*
 * UART stop bits
 */
typedef enum {
    STOP_BITS_1,
    STOP_BITS_2
} hal_uart_stop_bits_t;

/*
 * UART flow control
 */
typedef enum {
    FLOW_CONTROL_DISABLED,
    FLOW_CONTROL_CTS,
    FLOW_CONTROL_RTS,
    FLOW_CONTROL_CTS_RTS
} hal_uart_flow_control_t;

/*
 * UART parity
 */
typedef enum {
    NO_PARITY,
    ODD_PARITY,
    EVEN_PARITY
} hal_uart_parity_t;

/*
 * UART mode
 */
typedef enum {
    MODE_TX,
    MODE_RX,
    MODE_TX_RX
} hal_uart_mode_t;

/*
 * UART configuration
 */
typedef struct {
    uint32_t                baud_rate;
    hal_uart_data_width_t   data_width;
    hal_uart_parity_t       parity;
    hal_uart_stop_bits_t    stop_bits;
    hal_uart_flow_control_t flow_control;
    hal_uart_mode_t         mode;
} uart_config_t;

typedef struct {
    uint8_t       port;   /* uart port */
    uart_config_t config; /* uart config */
    void         *priv;   /* priv data */
} uart_dev_t;





static uart_dev_t at_uart;
static int at_uart_fd = -1;




static int read_and_discard_all_data(const int fd)
{
    int was_msg_already_printed = 0;
    int errno_code;

    while (1) {
        char buffer[1024];
        const ssize_t read_count = read(fd, buffer, sizeof(buffer));

        if (read_count == 0) {
            /* "EOF" or "connection closed at the other end"*/
            return 0;
        }

        if (read_count > 0) {
            if (!was_msg_already_printed) {
                printf("Some stale data was discarded.\r\n");
                was_msg_already_printed = 1;
            }

            continue;
        }

        assert(read_count == -1);  /* According to the specification. */

        errno_code = errno;

        if (errno_code == EINTR) {
            continue;
        }

        if (errno_code == EAGAIN ||
            errno_code == EWOULDBLOCK) {
            /**
             * We know that the file descriptor has been opened with
             * O_NONBLOCK or O_NDELAY, and these codes mean that there
             * is no data to read at present.
             */
            return 0;
        }

        /* Some other error has occurred. */
        return -1;
    }
}

static int32_t HAL_AT_Uart_Init(uart_dev_t *uart)
{
    int fd;
    struct termios t_opt;
    speed_t baud;

    if (uart->port != AT_UART_PORT) {
        return 0;
    }

    if ((at_uart_fd = open(AT_UART_LINUX_DEV,
                           O_RDWR | O_NOCTTY | O_NDELAY)) == -1) {
        printf("open at uart failed\r\n");
        return -1;
    }

    switch (uart->config.baud_rate) {
        case 115200:
            baud = B115200;
            break;
        case 921600:
            baud = B921600;
            break;
        default:
            baud = B115200;
            break;
    }

    fd = at_uart_fd;
    /* set the serial port parameters */
    fcntl(fd, F_SETFL, 0);
    if (0 != tcgetattr(fd, &t_opt)) {
        return -1;
    }

    if (0 != cfsetispeed(&t_opt, baud)) {
        return -1;
    }

    if (0 != cfsetospeed(&t_opt, baud)) {
        return -1;
    }

    /* 8N1, flow control, etc. */
    t_opt.c_cflag |= (CLOCAL | CREAD);
    if (uart->config.parity == NO_PARITY) {
        t_opt.c_cflag &= ~PARENB;
    }
    if (uart->config.stop_bits == STOP_BITS_1) {
        t_opt.c_cflag &= ~CSTOPB;
    } else {
        t_opt.c_cflag |= CSTOPB;
    }
    t_opt.c_cflag &= ~CSIZE;
    switch (uart->config.data_width) {
        case DATA_WIDTH_5BIT:
            t_opt.c_cflag |= CS5;
            break;
        case DATA_WIDTH_6BIT:
            t_opt.c_cflag |= CS6;
            break;
        case DATA_WIDTH_7BIT:
            t_opt.c_cflag |= CS7;
            break;
        case DATA_WIDTH_8BIT:
            t_opt.c_cflag |= CS8;
            break;
        default:
            t_opt.c_cflag |= CS8;
            break;
    }
    t_opt.c_lflag &= ~(ECHO | ECHOE | ISIG | ICANON);

    /**
     * AT is going to use a binary protocol, so make sure to
     * turn off any CR/LF translation and the like.
     */
    t_opt.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);

    t_opt.c_oflag &= ~OPOST;
    t_opt.c_cc[VMIN] = 0;
    t_opt.c_cc[VTIME] = 5;

    if (0 != tcsetattr(fd, TCSANOW, &t_opt)) {
        return -1;
    }

    printf("open at uart succeed\r\n");

    /* clear uart buffer */
    read_and_discard_all_data(fd);

    return 0;
}

static int32_t HAL_AT_Uart_Deinit(uart_dev_t *uart)
{
    if (uart->port == AT_UART_PORT) {
        close(at_uart_fd);
    }
    return 0;
}

static int32_t HAL_AT_Uart_Send(uart_dev_t *uart, const void *data,
                         uint32_t size, uint32_t timeout)
{
    uint32_t ret, rmd = size;

    if (uart->port == AT_UART_PORT) {
        while (rmd > 0) {
            ret = write(at_uart_fd, data + size - rmd, rmd);
            if (ret == -1) {
                printf("write uart fd failed on error: %d.\r\n", errno);
                return -1;
            }
            rmd -= ret;
        }
    } else {
        if (write(1, data, size) < 0) {
            printf("write failed\n");
        }
    }

    return 0;
}

static int32_t HAL_AT_Uart_Recv(uart_dev_t *uart, void *data, uint32_t expect_size,
                         uint32_t *recv_size, uint32_t timeout)
{
    int fd, n;

    if (uart->port == AT_UART_PORT) {
        fd = at_uart_fd;
    } else {
        fd = 1;
    }

    if ((n = read(fd, data, expect_size)) == -1) {
        return -1;
    }

    if (uart->port != AT_UART_PORT && *(char *)data == '\n') {
        *(char *)data = '\r';
    }
    if (recv_size) {
        *recv_size = n;
    }

    return 0;
}

static void at_uart_configure(uart_dev_t *u)
{
    u->port                = AT_UART_PORT;
    u->config.baud_rate    = AT_UART_BAUDRATE;
    u->config.data_width   = AT_UART_DATA_WIDTH;
    u->config.parity       = AT_UART_PARITY;
    u->config.stop_bits    = AT_UART_STOP_BITS;
    u->config.flow_control = AT_UART_FLOW_CONTROL;
    u->config.mode         = AT_UART_MODE;
}

static int at_init_uart()
{
    at_uart_configure(&at_uart);

    if (HAL_AT_Uart_Init(&at_uart) != 0) {
        return -1;
    }

    //at._pstuart = &at_uart;

    return 0;
}
/******************************************************/









/**
 * @brief Deallocate memory block
 *
 * @param[in] ptr @n Pointer to a memory block previously allocated with platform_malloc.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_Free(void *ptr)
{
    free(ptr);
}


int HAL_SetProductKey(char *product_key)
{
    int len = strlen(product_key);

    if (len > IOTX_PRODUCT_KEY_LEN) {
        return -1;
    }
    memset(_product_key, 0x0, IOTX_PRODUCT_KEY_LEN + 1);
    strncpy(_product_key, product_key, len);

    return len;
}


int HAL_SetProductSecret(char *product_secret)
{
    int len = strlen(product_secret);

    if (len > IOTX_PRODUCT_SECRET_LEN) {
        return -1;
    }
    memset(_product_secret, 0x0, IOTX_PRODUCT_SECRET_LEN + 1);
    strncpy(_product_secret, product_secret, len);

    return len;
}


int HAL_SetDeviceName(char *device_name)
{
    int len = strlen(device_name);

    if (len > IOTX_DEVICE_NAME_LEN) {
        return -1;
    }
    memset(_device_name, 0x0, IOTX_DEVICE_NAME_LEN + 1);
    strncpy(_device_name, device_name, len);

    return len;
}



int HAL_SetDeviceSecret(char *device_secret)
{
    int len = strlen(device_secret);

    if (len > IOTX_DEVICE_SECRET_LEN) {
        return -1;
    }
    memset(_device_secret, 0x0, IOTX_DEVICE_SECRET_LEN + 1);
    strncpy(_device_secret, device_secret, len);

    return len;
}




/**
 * @brief Get device name from user's system persistent storage
 *
 * @param [ou] device_name: array to store device name, max length is IOTX_DEVICE_NAME_LEN
 * @return the actual length of device name
 */
int HAL_GetDeviceName(char device_name[IOTX_DEVICE_NAME_LEN + 1])
{
	int len = strlen(_device_name);
	memset(device_name, 0, IOTX_DEVICE_NAME_LEN + 1);
	strncpy(device_name, _device_name, len);
	return strlen(device_name);
}

/**
 * @brief Get device secret from user's system persistent storage
 *
 * @param [ou] device_secret: array to store device secret, max length is IOTX_DEVICE_SECRET_LEN
 * @return the actual length of device secret
 */
int HAL_GetDeviceSecret(char device_secret[IOTX_DEVICE_SECRET_LEN + 1])
{
	int len = strlen(_device_secret);
    memset(device_secret, 0x0, IOTX_DEVICE_SECRET_LEN + 1);

    strncpy(device_secret, _device_secret, len);

    return len;
}

/**
 * @brief Get firmware version
 *
 * @param [ou] version: array to store firmware version, max length is IOTX_FIRMWARE_VER_LEN
 * @return the actual length of firmware version
 */
int HAL_GetFirmwareVersion(char *version)
{
	char *ver = "app-1.0.0-20190812.1000";
    int len = strlen(ver);
    memset(version, 0x0, IOTX_FIRMWARE_VER_LEN);
    strncpy(version, ver, IOTX_FIRMWARE_VER_LEN);
    version[len] = '\0';
    return strlen(version);
}


/**
 * @brief Get product key from user's system persistent storage
 *
 * @param [ou] product_key: array to store product key, max length is IOTX_PRODUCT_KEY_LEN
 * @return  the actual length of product key
 */
int HAL_GetProductKey(char product_key[IOTX_PRODUCT_KEY_LEN + 1])
{
	int len = strlen(_product_key);
    memset(product_key, 0x0, IOTX_PRODUCT_KEY_LEN + 1);

    strncpy(product_key, _product_key, len);

    return len;
}


int HAL_GetProductSecret(char product_secret[IOTX_PRODUCT_SECRET_LEN + 1])
{
	int len = strlen(_product_secret);
    memset(product_secret, 0x0, IOTX_PRODUCT_SECRET_LEN + 1);

    strncpy(product_secret, _product_secret, len);

    return len;
}


/**
 * @brief Allocates a block of size bytes of memory, returning a pointer to the beginning of the block.
 *
 * @param [in] size @n specify block size in bytes.
 * @return A pointer to the beginning of the block.
 * @see None.
 * @note Block value is indeterminate.
 */
void *HAL_Malloc(uint32_t size)
{
	return malloc(size);
}


/**
 * @brief Create a mutex.
 *
 * @retval NULL : Initialize mutex failed.
 * @retval NOT_NULL : The mutex handle.
 * @see None.
 * @note None.
 */
void *HAL_MutexCreate(void)
{
	int err_num;
    pthread_mutex_t *mutex = (pthread_mutex_t *)HAL_Malloc(sizeof(pthread_mutex_t));
    if (NULL == mutex) {
        return NULL;
    }

    if (0 != (err_num = pthread_mutex_init(mutex, NULL))) {
        printf("create mutex failed\n");
        HAL_Free(mutex);
        return NULL;
    }

    return mutex;
}

/**
 * @brief Destroy the specified mutex object, it will release related resource.
 *
 * @param [in] mutex @n The specified mutex.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_MutexDestroy(void *mutex)
{
    int err_num;

    if (!mutex) {
        printf("mutex want to destroy is NULL!\n");
        return;
    }
    if (0 != (err_num = pthread_mutex_destroy((pthread_mutex_t *)mutex))) {
        printf("destroy mutex failed\n");
    }

    HAL_Free(mutex);
}


/**
 * @brief Waits until the specified mutex is in the signaled state.
 *
 * @param [in] mutex @n the specified mutex.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_MutexLock(void *mutex)
{
	int err_num;
    if (0 != (err_num = pthread_mutex_lock((pthread_mutex_t *)mutex))) {
        printf("lock mutex failed: - '%s' (%d)\n", strerror(err_num), err_num);
    }
}

/**
 * @brief Releases ownership of the specified mutex object..
 *
 * @param [in] mutex @n the specified mutex.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_MutexUnlock(void *mutex)
{
	int err_num;
    if (0 != (err_num = pthread_mutex_unlock((pthread_mutex_t *)mutex))) {
        printf("unlock mutex failed - '%s' (%d)\n", strerror(err_num), err_num);
    }
}
/**
 * @brief Writes formatted data to stream.
 *
 * @param [in] fmt: @n String that contains the text to be written, it can optionally contain embedded format specifiers
     that specifies how subsequent arguments are converted for output.
 * @param [in] ...: @n the variable argument list, for formatted and inserted in the resulting string replacing their respective specifiers.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_Printf(const char *fmt, ...)
{
	va_list args;

    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    fflush(stdout);
}


uint32_t HAL_Random(uint32_t region)
{
	FILE *handle;
    ssize_t ret = 0;
    uint32_t output = 0;
    handle = fopen("/dev/urandom","r");
    if (handle == NULL) {
        printf("open /dev/urandom failed\n");
        return 0;
    }
    ret = fread(&output,sizeof(uint32_t),1,handle);
    if (ret != 1) {
        printf("fread error: %ld\n",ret);
        fclose(handle);
        return 0;
    }
    fclose(handle);
    return (region > 0) ? (output % region) : 0;
}


/**
 * @brief Sleep thread itself.
 *
 * @param [in] ms @n the time interval for which execution is to be suspended, in milliseconds.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_SleepMs(uint32_t ms)
{
	usleep(1000 * ms);
}


/**
 * @brief Writes formatted data to string.
 *
 * @param [out] str: @n String that holds written text.
 * @param [in] len: @n Maximum length of character will be written
 * @param [in] fmt: @n Format that contains the text to be written, it can optionally contain embedded format specifiers
     that specifies how subsequent arguments are converted for output.
 * @param [in] ...: @n the variable argument list, for formatted and inserted in the resulting string replacing their respective specifiers.
 * @return bytes of character successfully written into string.
 * @see None.
 * @note None.
 */
int HAL_Snprintf(char *str, const int len, const char *fmt, ...)
{
	va_list args;
    int     rc;

    va_start(args, fmt);
    rc = vsnprintf(str, len, fmt, args);
    va_end(args);

    return rc;
}


void HAL_Srandom(uint32_t seed)
{
	srandom(seed);
}


/**
 * @brief Destroy the specific TCP connection.
 *
 * @param [in] fd: @n Specify the TCP connection by handle.
 *
 * @return The result of destroy TCP connection.
 * @retval < 0 : Fail.
 * @retval   0 : Success.
 */
    int HAL_TCP_Destroy(uintptr_t fd)
{
	if(HAL_AT_Uart_Deinit(&at_uart) == 0){
		printf("close uart success.\n");
		return 0;
	}{
		printf("close usart error.\n");
		return -1;
	}
}


/**
 * @brief Establish a TCP connection.
 *
 * @param [in] host: @n Specify the hostname(IP) of the TCP server
 * @param [in] port: @n Specify the TCP port of TCP server
 *
 * @return The handle of TCP connection.
   @retval   0 : Fail.
   @retval > 0 : Success, the value is handle of this TCP connection.
 */
    uintptr_t HAL_TCP_Establish(const char *host, uint16_t port)
{
	if(at_init_uart() == 0){
		printf("open uart ok:%d\n", at_uart_fd);
		return (uintptr_t)at_uart_fd;
	}else{
		printf("open uart error.\n");
		return 0;
	}
}


/**
 * @brief Read data from the specific TCP connection with timeout parameter.
 *        The API will return immediately if 'len' be received from the specific TCP connection.
 *
 * @param [in] fd @n A descriptor identifying a TCP connection.
 * @param [out] buf @n A pointer to a buffer to receive incoming data.
 * @param [out] len @n The length, in bytes, of the data pointed to by the 'buf' parameter.
 * @param [in] timeout_ms @n Specify the timeout value in millisecond. In other words, the API block 'timeout_ms' millisecond maximumly.
 *
 * @retval       -2 : TCP connection error occur.
 * @retval       -1 : TCP connection be closed by remote server.
 * @retval        0 : No any data be received in 'timeout_ms' timeout period.
 * @retval (0, len] : The total number of bytes be received in 'timeout_ms' timeout period.

 * @see None.
 */
int32_t HAL_TCP_Read(uintptr_t fd, char *buf, uint32_t len, uint32_t timeout_ms)
{
	uint32_t recv_size;	
	if(HAL_AT_Uart_Recv(&at_uart, buf, len, &recv_size, timeout_ms) == 0){
		return recv_size;
	}else{
		printf("tcp read error.\n");
		return -2;
	}
}


/**
 * @brief Write data into the specific TCP connection.
 *        The API will return immediately if 'len' be written into the specific TCP connection.
 *
 * @param [in] fd @n A descriptor identifying a connection.
 * @param [in] buf @n A pointer to a buffer containing the data to be transmitted.
 * @param [in] len @n The length, in bytes, of the data pointed to by the 'buf' parameter.
 * @param [in] timeout_ms @n Specify the timeout value in millisecond. In other words, the API block 'timeout_ms' millisecond maximumly.
 *
 * @retval      < 0 : TCP connection error occur..
 * @retval        0 : No any data be write into the TCP connection in 'timeout_ms' timeout period.
 * @retval (0, len] : The total number of bytes be written in 'timeout_ms' timeout period.

 * @see None.
 */
    int32_t HAL_TCP_Write(uintptr_t fd, const char *buf, uint32_t len, uint32_t timeout_ms)
{
	if(HAL_AT_Uart_Send(&at_uart, buf, len, timeout_ms) == 0){
		return len;
	}else{
		printf("tcp write error.\n");
		return -1;
	}
}


/**
 * @brief Retrieves the number of milliseconds that have elapsed since the system was boot.
 *
 * @return the number of milliseconds.
 * @see None.
 * @note None.
 */
uint64_t HAL_UptimeMs(void)
{
    uint64_t            time_ms;
    struct timespec     ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    time_ms = ((uint64_t)ts.tv_sec * (uint64_t)1000) + (ts.tv_nsec / 1000 / 1000);

    return time_ms;
}

int HAL_Vsnprintf(char *str, const int len, const char *format, va_list ap)
{
    return vsnprintf(str, len, format, ap);
}

