/*
 *	Module Name:	SPI_CMD.H
 *	Abstract:	Define the constant for mcp2510's spi command interface
 *	Environment:	MCP2510
 *	Date: 		2011/09/18
 *	By: 		Staufen Yang
 */ 

#ifndef	SPI_CMD_H
#define	SPI_CMD_H

#include "can.h"
#include "mcpcan.h"

/*	 Define the command constant	*/
#define	SPI_CMD_READ			(0x03)
#define	SPI_CMD_WRITE			(0x02)
#define	SPI_CMD_RTS(n)			(0x80 + (1<<n))
#define	SPI_CMD_READSTA			(0xa0)
#define	SPI_CMD_BITMOD			(0x05)
#define	SPI_CMD_RESET			(0xc0)

/*	define function access	*/
#ifndef	_SPI_CMD_SOURCE_
#define	SPI_CMD_EXTERN	extern
#else
#define SPI_CMD_EXTERN
#endif
SPI_CMD_EXTERN U8 mcp2510_read_reg( struct mcp251x *chip, U8 addr);
SPI_CMD_EXTERN VOID mcp2510_write_reg( struct mcp251x *chip, U8 addr, BYTE data);
SPI_CMD_EXTERN VOID mcp2510_modify_bit( struct mcp251x *chip, U8 addr, U8 mask, U8 data);
SPI_CMD_EXTERN VOID mcp2510_reset( struct mcp251x *chip);
SPI_CMD_EXTERN VOID mcp2510_hw_reset( struct mcp251x *chip);
SPI_CMD_EXTERN VOID mcp2510_rts(struct mcp251x *chip, int buf_idx);
SPI_CMD_EXTERN BOOL mcp2510_set_bit_rate(struct mcp251x *chip, int bit_rate);
//SPI_CMD_EXTERN BOOL mcp2510_set_filter( struct mcp251x *chip, struct can_filter *filter);
SPI_CMD_EXTERN BOOL mcp2510_send_frame( struct mcp251x *chip, int buf_idx);
SPI_CMD_EXTERN BOOL mcp2510_read_frame( struct mcp251x *chip, int buf_idx);

#endif
