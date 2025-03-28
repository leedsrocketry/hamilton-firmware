/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Alexandra Posta
  Created on: 22 March 2025
  Description: Data storage abstraction layer
*/

#include "datastore.h"

void init_datastore()
{
    MAX_STORAGE_CAPACITY = NAND_STORAGE_CAPACITY;
    CURRENT_STORAGE_USED = 0;
}

void read()
{
  
}

void write(Frame frame)
{
    log_frame(frame);
}