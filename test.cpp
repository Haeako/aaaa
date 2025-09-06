#include<lib/point/point.h>
 #include <unistd.h>

int main()
{
    Reader point_r;
    int x , y;
    while (true)
    {
        point_r.get_coor(&x, &y);
        std::cout <<  x << "   "<< y << std::endl;
        sleep(1);
    }

}