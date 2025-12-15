/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*       Solutions for real time microcontroller applications         *
**********************************************************************
*       SEGGER RTT * Real Time Transfer for embedded targets         *
**********************************************************************
*/

#ifndef SEGGER_RTT_H
#define SEGGER_RTT_H

#include "SEGGER_RTT_Conf.h"
#include <stdarg.h>

/*********************************************************************
*       Defines, fixed
*/
#define SEGGER_RTT_MODE_NO_BLOCK_SKIP         (0)     // Skip if buffer full
#define SEGGER_RTT_MODE_NO_BLOCK_TRIM         (1)     // Trim output to fit buffer
#define SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL    (2)     // Block if buffer full

/*********************************************************************
*       Function prototypes
*/
void          SEGGER_RTT_Init            (void);
unsigned      SEGGER_RTT_Write           (unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned      SEGGER_RTT_WriteString     (unsigned BufferIndex, const char* s);
int           SEGGER_RTT_printf          (unsigned BufferIndex, const char* sFormat, ...);
int           SEGGER_RTT_vprintf         (unsigned BufferIndex, const char* sFormat, va_list* pParamList);
unsigned      SEGGER_RTT_Read            (unsigned BufferIndex, void* pBuffer, unsigned BufferSize);
int           SEGGER_RTT_HasKey          (void);
int           SEGGER_RTT_GetKey          (void);
int           SEGGER_RTT_WaitKey         (void);
int           SEGGER_RTT_ConfigUpBuffer  (unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags);
int           SEGGER_RTT_ConfigDownBuffer(unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags);
int           SEGGER_RTT_SetTerminal     (unsigned char TerminalId);
int           SEGGER_RTT_TerminalOut     (unsigned char TerminalId, const char* s);

#endif
