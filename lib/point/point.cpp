#include "point.h"

Reader::Reader() {
  shm_fd = shm_open(name, O_RDONLY,0666);
  if (shm_fd == -1){
    std::cerr << "[E] Failed to open shared memory\n";
  }
  ptr = mmap(0, SIZE, PROT_READ, MAP_SHARED, shm_fd, 0);
  if(ptr == MAP_FAILED){
    std::cerr << "[E] Failed to map " << std::endl;
  }
};

void Reader::get_coor(int *x, int *y) {
 // cast nums ptr to extrac data
 int flag = 0; 
 nums = (int*)ptr;
  *x = nums[0];
  *y = nums[1];
  flag = nums[2];
  if (!flag)
  {
    printf("Revicce end transmit signal from camera process\n");
    *x = 0 ; *y = 0;
  }
  
  return;
};

Reader::~Reader()
{
  munmap(ptr, SIZE);
  close(shm_fd);
};
        
