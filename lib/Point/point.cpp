#include <point.h>

Point getPoint(Point &buffer)
  {
  String input = "";
  while (Serial.available())
    input = Serial.readStringUntil('\r');  
  if (input == "")
    {
      buffer.x = 0;
      buffer.y = 0;
    }
  else
  {
    int spaceIndex= input.indexOf(' ');

    // Tách và chuyển đổi số đầu tiên thành int
    buffer.x= input.substring(0, spaceIndex).toFloat();
   if(!Serial.available())
   {
    //Serial.println("NOT available");
   }
    // Tách và chuyển đổi số thứ hai thành int
    buffer.y = input.substring(spaceIndex + 1, input.length()).toFloat();
  }
  return buffer;
  }
