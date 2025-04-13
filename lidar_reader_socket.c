#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>

#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"
#define DEFAULT_MAX_POINTS 1000
#define SOCKET_PATH "/tmp/lidar_socket"
#define FRAME_BUFFER_SIZE 65536
#define DEBUG 0

char last_frame[FRAME_BUFFER_SIZE];
pthread_mutex_t frame_mutex = PTHREAD_MUTEX_INITIALIZER;

long long current_millis() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000;
}

int configure_serial(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}

int read_byte(int fd) {
    unsigned char c;
    int n = read(fd, &c, 1);
    return (n == 1) ? c : -1;
}

void* serial_reader_thread(void* arg) {
    const char* serial_port = (const char*)arg;
    int max_points = DEFAULT_MAX_POINTS;
    float *angles = malloc(sizeof(float) * max_points);
    int *dists = malloc(sizeof(int) * max_points);
    if (!angles || !dists) {
        perror("malloc");
        exit(1);
    }

    int serial_fd = open(serial_port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        perror("open serial");
        exit(1);
    }

    if (configure_serial(serial_fd) != 0) {
        close(serial_fd);
        exit(1);
    }

    int count = 0;
    static int frame_count = 0;

    while (1) {
        int c = read_byte(serial_fd);
        if (c != 0xAA) continue;
        if (read_byte(serial_fd) != 0x55) continue;

        int CT = read_byte(serial_fd);
        int LSN = read_byte(serial_fd);
        int FSA = read_byte(serial_fd) | (read_byte(serial_fd) << 8);
        int LSA = read_byte(serial_fd) | (read_byte(serial_fd) << 8);
        read_byte(serial_fd);  // CS (ignore)
        read_byte(serial_fd);

        float F = (FSA >> 1) / 64.0;
        float L = (LSA >> 1) / 64.0;

        for (int i = 0; i < LSN; ++i) {
            int Si = read_byte(serial_fd) | (read_byte(serial_fd) << 8);
            int dist = Si >> 2;
            float A_corr = (dist == 0) ? 0.0 : atan(19.16 * (dist - 90.15) / (dist * 90.15));
            float angle = F + ((L - F) / LSN) * i - A_corr;

            if (count < max_points) {
                angles[count] = angle;
                dists[count] = dist;
                count++;
            }
        }

        if (CT == 1) {
            long long ts = current_millis();

            pthread_mutex_lock(&frame_mutex);
            int offset = 0;
            for (int i = 0; i < count && offset < FRAME_BUFFER_SIZE - 32; ++i) {
                offset += snprintf(last_frame + offset, FRAME_BUFFER_SIZE - offset, "%.2f,%d\n", angles[i], dists[i]);
            }
            snprintf(last_frame + offset, FRAME_BUFFER_SIZE - offset, "NEWFRAME %lld\n", ts);
            pthread_mutex_unlock(&frame_mutex);

            frame_count++;
            if (DEBUG && frame_count % 20 == 0) {
                printf("[Serial] Frame %d armazenado (%d pontos)\n", frame_count, count);
                fflush(stdout);
            }

            count = 0;
        }
    }

    close(serial_fd);
    free(angles);
    free(dists);
    return NULL;
}

int main(int argc, char *argv[]) {
    const char* serial_port = (argc > 1) ? argv[1] : DEFAULT_SERIAL_PORT;

    unlink(SOCKET_PATH);

    int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd == -1) {
        perror("socket");
        return 1;
    }

    struct sockaddr_un addr = {0};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        perror("bind");
        return 1;
    }

    if (listen(server_fd, 5) == -1) {
        perror("listen");
        return 1;
    }

    pthread_t reader_thread;
    pthread_create(&reader_thread, NULL, serial_reader_thread, (void*)serial_port);

    printf("LidarReader rodando em %s\n", SOCKET_PATH);

    while (1) {
        int client_fd = accept(server_fd, NULL, NULL);
        if (client_fd == -1) {
            perror("accept");
            continue;
        }

        char buffer[16];
        ssize_t n = read(client_fd, buffer, sizeof(buffer));
        if (n > 0) {
            if (DEBUG) {
                printf("[Server] Cliente conectou e pediu frame\n");
            }
            pthread_mutex_lock(&frame_mutex);
            write(client_fd, last_frame, strlen(last_frame));
            pthread_mutex_unlock(&frame_mutex);
        } else {
            if (DEBUG) {
                printf("[Server] Cliente conectou mas nao enviou nada (n=%zd)\n", n);
            }
        }
        fflush(stdout);

        close(client_fd);
    }

    close(server_fd);
    unlink(SOCKET_PATH);
    return 0;
}
