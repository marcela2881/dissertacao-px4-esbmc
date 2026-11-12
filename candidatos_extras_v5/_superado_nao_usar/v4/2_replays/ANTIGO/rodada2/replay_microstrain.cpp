#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define PORT_SIZE 128
int main() {
    char *_port = (char*)malloc(PORT_SIZE);
    memset(_port, 0, PORT_SIZE);
    char uart_port[PORT_SIZE + 1];
    memset(uart_port, 'A', PORT_SIZE);
    uart_port[PORT_SIZE] = '\0';
    size_t max_len = PORT_SIZE;
    strncpy(_port, uart_port, max_len);
    printf("Tentando escrever em _port[%zu]...\n", max_len);
    _port[max_len] = '\0';
    printf("Escreveu\n");
    free(_port);
    return 0;
}
