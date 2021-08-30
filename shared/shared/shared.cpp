#include "shared.h"

void fill_checksum(char *p, int sz){
  p[sz - 1] = 0;
  char tmp = 0;
  for(int i =0; i < sz; i++){
    tmp += p[i];
  }
  p[sz - 1]= tmp;
}

int check_checksum(char *p, int sz){
  char check = p[sz - 1];
  p[sz - 1] = 0;
  char tmp = 0;
  for(int i =0; i < sz; i++){
    tmp += p[i];
  }
  p[sz - 1] = check;
  return tmp == check;
}