// comunicate.h
#ifndef COMUNICATE_H
#define COMUNICATE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#define FIFO_PATH "/tmp/stepper_fifo"
#define BUF_SIZE 128

class FifoServer {
private:
    int fd;
    int lastX;
    int lastY;

public:
    FifoServer() : lastX(0), lastY(0) {        
        // Open FIFO in non-blocking mode
        fd = open(FIFO_PATH, O_RDWR);
        if (fd == -1) {
            std::cerr << "Error opening FIFO: " << strerror(errno) << std::endl;
        }
    }

    ~FifoServer() {
        if (fd != -1) close(fd);
    }

    // Modified run method to update Point coordinates
    void run(int &x, int &y) {
        char buffer[BUF_SIZE] = {0};
        int bytesReceived = read(fd, buffer, BUF_SIZE - 1);
        
        if (bytesReceived > 0) {
            std::stringstream ss(buffer);
            std::string token;
            
            // Parse X coordinate
            if (std::getline(ss, token, ' ')) {
                try {
                    lastX = std::stoi(token);
                } catch (const std::exception& e) {
                    std::cerr << "Error parsing X coordinate: " << e.what() << std::endl;
                    return;
                }
            }
            
            // Parse Y coordinate
            // Parse Y coordinate
            if (std::getline(ss, token, ' ')) {
                try {
                    lastY = std::stoi(token);
                } catch (const std::exception& e) {
                    std::cerr << "Error parsing Y coordinate: " << e.what() << std::endl;
                    return;
                }
            }
        }
        
        // Update output parameters
        x = lastX;
        y = lastY;
    }
};

#endif // COMUNICATE_H
