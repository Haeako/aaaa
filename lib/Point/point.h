#include <Arduino.h>

typedef struct 
{
  int16_t x;
  int16_t y;
} Point;

Point getPoint(Point &buffer);