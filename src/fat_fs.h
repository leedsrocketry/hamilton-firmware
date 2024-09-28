//
// Created by Morgan Thomas on 09/08/2024.
//

#ifndef HAMILTON_FIRMWARE_FAT_FS_H
#define HAMILTON_FIRMWARE_FAT_FS_H

#include "mcu.h"




typedef struct {
    __IO uint8_t BS_jmpBoot[3];
    __IO char BS_OEMName[8];
    __IO uint16_t BPB_BytsPerSec;
    __IO uint8_t BPB_SecPerClus;
    __IO uint16_t BPB_RsvdSecCnt;
    __IO uint8_t BPB_NumFATs;
    __IO uint16_t BPB_RootEntCnt;
    __IO uint16_t BPB_TotSec16;
    __IO uint8_t BPB_Media;
    __IO uint16_t BPB_FATSz16;
    __IO uint16_t BPB_SecPerTrk;
    __IO uint16_t BPB_NumHeads;
    __IO uint32_t BPB_HiddSec;
    __IO uint32_t BPB_TotSec32;
    // FAT32 extended BPB
    __IO uint32_t BPB_FATSz32;
    __IO uint16_t BPB_ExtFlags;
    __IO uint16_t BPB_FSVer;
    __IO uint32_t BPB_RootClus;
    __IO uint16_t BPB_FSInfo;
    __IO uint16_t BPB_BkBootSec
    __IO uint8_t BPB_Reserved[12];
    __IO uint8_t BS_DrvNum;
    __IO uint8_t BS_Reserved1;
    __IO uint8_t BS_BootSig;
    __IO uint32_t BS_VolID;
    __IO char BS_VolLab[11];
    __IO char BS_FilSysType[8]
    __IO uint8_t reserved[420];
    __IO uint16_t Signature_word;


}  __attribute__((__packed__)) FAT_BootSector;

uint8_t retrieve_boot_sector(SD_Card device,FAT_BootSector *boot_sector);



#endif //HAMILTON_FIRMWARE_FAT_FS_H
