// Copyright (c) 2016, XMOS Ltd, All rights reserved
#include <xs1.h>
#include <stdio.h>
#include <stdlib.h>
#include "control.h"

int main(void)
{
  if (control_init() != CONTROL_SUCCESS) printf("ERROT on control_init\n");
  else printf("Success!\n");
  return 1;
}
