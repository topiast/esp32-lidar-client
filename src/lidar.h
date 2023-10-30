#pragma once

#include <vector>
#include <HardwareSerial.h>
#include <Arduino.h>


#include "point.h"

class Lidar {
private:
    const size_t max_buffer_size = 300;
    size_t buffer_index = 0;
    std::vector<Point2d> points_buffer;
    HardwareSerial SerialPort;
public:
    Lidar() : SerialPort(2), points_buffer(max_buffer_size) {}
    void begin(const int lidarBaudRate, const int rxPin = 16, const int txPin = 17);
    size_t scan();
    Point2d* begin_points() { return points_buffer.data(); };
    size_t points_added() { return buffer_index; };
    float get_buffer_fill() { return float(buffer_index) / float(max_buffer_size); };
    void clear_buffer() { buffer_index = 0; };
};

