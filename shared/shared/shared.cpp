#include "shared.h"

void fill_checksum(char *p, int sz){
  p[sz - 1] = 0;
  p[sz - 2] = 0;

  char tmp = 0;
  for(int i =0; i < sz; i++){
    tmp += p[i];
  }
  p[sz - 2]= tmp;

  tmp = 0;
  for(int i =0; i < sz; i++){
    tmp ^= p[i];
  }

  p[sz - 1]= tmp;
}

int check_checksum(char *p, int sz){
  char check_1 = p[sz - 2];
  char check_2 = p[sz - 1];

  p[sz - 1] = 0;
  p[sz - 2] = 0;

  char tmp = 0;
  for(int i =0; i < sz; i++){
    tmp += p[i];
  }
  p[sz - 2]= tmp;

  tmp = 0;
  for(int i =0; i < sz; i++){
    tmp ^= p[i];
  }

  p[sz - 1] = tmp;

  return (p[sz - 2] == check_1) && (p[sz - 1] == check_2);
}