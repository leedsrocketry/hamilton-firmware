/*
  Leeds University Rocketry Organisation - LURA
  Author Name: Evan Madurai
  Created on: 29 Mar 2025
  Description:
*/

#include "datastore.h"

void init_datastore() {
  MAX_STORAGE_CAPACITY = NAND_STORAGE_CAPACITY;
  CURRENT_STORAGE_USED = 0;
}

void read() {}

void write(Frame frame) { save_frame(frame); }