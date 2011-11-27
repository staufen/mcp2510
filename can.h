/*
 * can.h
 * Define the type and the structure of CAN frame
 * Date	: 2011/09/17
 * By	: Staufen Yang
 */

#ifndef CAN_H
#define CAN_H

#include <linux/types.h>

#define U32 unsigned int
#define U16 unsigned short
#define S32 int
#define S16 short int
#define U8  unsigned char
#define S8  char

#define UINT16 U16
#define UINT32 U32
#define UINT8  U8

#define TRUE 	1   
#define FALSE 	0

#define BOOL   int
#define VOID   void

#define BYTE   U8
#define WORD   S16
#define DWORD  S32

/**
 * struct can_frame - basic CAN frame structure
 * @info_id:  indicate what does this can frame use for
 * @src_addr:  the source address of this can frame
 * @dest_addr:  the destination address of this can frame
 * @info_order:  indicate the order of this can frame in the whole can information sequence
 * @can_dlc: the data length field of the CAN frame
 * @data:    the CAN frame payload.
 */
#define CAN_FRAME_MAX_DATA_LEN 8

struct can_frame {
	BYTE    info_id;
	BYTE    src_addr;
	BYTE    dest_addr;
	BYTE    info_order;
	BYTE    can_dlc; /* data length code: 0 .. 8 */
//	__u8    data[8] __attribute__((aligned(8)));
	BYTE	data[8] __attribute__((aligned(8)));
};



#endif /* CAN_H */
