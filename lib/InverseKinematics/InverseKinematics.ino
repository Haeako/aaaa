#include "InverseKinematics.h"
/*  machine(d, e, f, g)
      d = distance from the center of the base to any of its corners
      e = distance from the center of the platform to any of its corners
      f = length of link #1
      g = length of link #2
*/
Machine machine(2.5, 2.5, 1.5, 3);

void setup() {
  Serial.begin(115200);
  Serial.println((String) "Theta A = " + machine.theta(A, 3.5, 0.2, 0.2) + " degrees");
  Serial.println((String) "Theta B = " + machine.theta(B, 3.5, 0.2, 0.2) + " degrees");
  Serial.println((String) "Theta C = " + machine.theta(C, 3.5, 0.2, 0.2) + " degrees");
}

void loop() {
}
