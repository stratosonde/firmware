/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*       Solutions for real time microcontroller applications         *
**********************************************************************
*       SEGGER RTT * Real Time Transfer for embedded targets         *
**********************************************************************
*/

#include "SEGGER_RTT.h"
#include <string.h>
#include <stdio.h>

/*********************************************************************
*       Types
*/
typedef struct {
  const char* sName;
  char*       pBuffer;
  unsigned    SizeOfBuffer;
  unsigned    WrOff;
  volatile unsigned RdOff;
  unsigned    Flags;
} SEGGER_RTT_BUFFER_UP;

typedef struct {
  const char* sName;
  char*       pBuffer;
  unsigned    SizeOfBuffer;
  volatile unsigned WrOff;
  unsigned    RdOff;
  unsigned    Flags;
} SEGGER_RTT_BUFFER_DOWN;

typedef struct {
  char                    acID[16];
  int                     MaxNumUpBuffers;
  int                     MaxNumDownBuffers;
  SEGGER_RTT_BUFFER_UP    aUp[SEGGER_RTT_MAX_NUM_UP_BUFFERS];
  SEGGER_RTT_BUFFER_DOWN  aDown[SEGGER_RTT_MAX_NUM_DOWN_BUFFERS];
} SEGGER_RTT_CB;

/*********************************************************************
*       Static data
*/
static char _acUpBuffer  [BUFFER_SIZE_UP];
static char _acDownBuffer[BUFFER_SIZE_DOWN];

SEGGER_RTT_CB _SEGGER_RTT = {
  "SEGGER RTT",
  SEGGER_RTT_MAX_NUM_UP_BUFFERS,
  SEGGER_RTT_MAX_NUM_DOWN_BUFFERS,
  {{ "Terminal", &_acUpBuffer[0],   sizeof(_acUpBuffer),   0, 0, SEGGER_RTT_MODE_DEFAULT }},
  {{ "Terminal", &_acDownBuffer[0], sizeof(_acDownBuffer), 0, 0, SEGGER_RTT_MODE_DEFAULT }}
};

/*********************************************************************
*       Static functions
*/
static unsigned _WriteNoCheck(SEGGER_RTT_BUFFER_UP* pRing, const char* pData, unsigned NumBytes) {
  unsigned NumBytesToWrite;
  unsigned WrOff;
  unsigned Rem;
  
  WrOff = pRing->WrOff;
  Rem = pRing->SizeOfBuffer - WrOff;
  if (Rem > NumBytes) {
    memcpy(pRing->pBuffer + WrOff, pData, NumBytes);
    pRing->WrOff = WrOff + NumBytes;
  } else {
    NumBytesToWrite = Rem;
    memcpy(pRing->pBuffer + WrOff, pData, NumBytesToWrite);
    NumBytesToWrite = NumBytes - Rem;
    memcpy(pRing->pBuffer, pData + Rem, NumBytesToWrite);
    pRing->WrOff = NumBytesToWrite;
  }
  return NumBytes;
}

static unsigned _GetAvailWriteSpace(SEGGER_RTT_BUFFER_UP* pRing) {
  unsigned RdOff;
  unsigned WrOff;
  unsigned r;
  
  RdOff = pRing->RdOff;
  WrOff = pRing->WrOff;
  if (RdOff <= WrOff) {
    r = pRing->SizeOfBuffer - 1u - WrOff + RdOff;
  } else {
    r = RdOff - WrOff - 1u;
  }
  return r;
}

/*********************************************************************
*       Public functions
*/
void SEGGER_RTT_Init(void) {
  // Already initialized statically
}

/* RTT_DISABLED: Stub out for low power testing - change to 0 to re-enable */
#define RTT_DISABLED 0

unsigned SEGGER_RTT_Write(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes) {
#if RTT_DISABLED
  /* Stub: Return immediately to save power */
  (void)BufferIndex;
  (void)pBuffer;
  return NumBytes;  /* Pretend we wrote everything */
#else
  unsigned NumBytesToWrite;
  unsigned NumBytesWritten;
  SEGGER_RTT_BUFFER_UP* pRing;
  
  pRing = &_SEGGER_RTT.aUp[BufferIndex];
  SEGGER_RTT_LOCK();
  NumBytesToWrite = _GetAvailWriteSpace(pRing);
  if (NumBytesToWrite < NumBytes) {
    if (pRing->Flags == SEGGER_RTT_MODE_NO_BLOCK_SKIP) {
      NumBytesWritten = 0u;
    } else {
      NumBytesWritten = _WriteNoCheck(pRing, (const char*)pBuffer, NumBytesToWrite);
    }
  } else {
    NumBytesWritten = _WriteNoCheck(pRing, (const char*)pBuffer, NumBytes);
  }
  SEGGER_RTT_UNLOCK();
  return NumBytesWritten;
#endif
}

unsigned SEGGER_RTT_WriteString(unsigned BufferIndex, const char* s) {
  return SEGGER_RTT_Write(BufferIndex, s, strlen(s));
}

int SEGGER_RTT_printf(unsigned BufferIndex, const char* sFormat, ...) {
  char ac[SEGGER_RTT_PRINTF_BUFFER_SIZE];
  va_list ParamList;
  int r;
  
  va_start(ParamList, sFormat);
  r = vsnprintf(ac, sizeof(ac), sFormat, ParamList);
  va_end(ParamList);
  if (r > 0) {
    SEGGER_RTT_Write(BufferIndex, ac, (unsigned)r);
  }
  return r;
}

int SEGGER_RTT_vprintf(unsigned BufferIndex, const char* sFormat, va_list* pParamList) {
  char ac[SEGGER_RTT_PRINTF_BUFFER_SIZE];
  int r;
  
  r = vsnprintf(ac, sizeof(ac), sFormat, *pParamList);
  if (r > 0) {
    SEGGER_RTT_Write(BufferIndex, ac, (unsigned)r);
  }
  return r;
}

unsigned SEGGER_RTT_Read(unsigned BufferIndex, void* pBuffer, unsigned BufferSize) {
  unsigned RdOff;
  unsigned WrOff;
  unsigned NumBytesRead;
  unsigned NumBytesRem;
  SEGGER_RTT_BUFFER_DOWN* pRing;
  char* pDst;
  
  pDst = (char*)pBuffer;
  pRing = &_SEGGER_RTT.aDown[BufferIndex];
  RdOff = pRing->RdOff;
  WrOff = pRing->WrOff;
  NumBytesRead = 0u;
  if (RdOff != WrOff) {
    if (WrOff > RdOff) {
      NumBytesRem = WrOff - RdOff;
      if (NumBytesRem > BufferSize) {
        NumBytesRem = BufferSize;
      }
      memcpy(pDst, pRing->pBuffer + RdOff, NumBytesRem);
      NumBytesRead = NumBytesRem;
      pRing->RdOff = RdOff + NumBytesRem;
    } else {
      NumBytesRem = pRing->SizeOfBuffer - RdOff;
      if (NumBytesRem > BufferSize) {
        NumBytesRem = BufferSize;
      }
      memcpy(pDst, pRing->pBuffer + RdOff, NumBytesRem);
      NumBytesRead = NumBytesRem;
      BufferSize -= NumBytesRem;
      pDst += NumBytesRem;
      RdOff = 0;
      if (BufferSize > 0) {
        NumBytesRem = WrOff;
        if (NumBytesRem > BufferSize) {
          NumBytesRem = BufferSize;
        }
        memcpy(pDst, pRing->pBuffer, NumBytesRem);
        NumBytesRead += NumBytesRem;
        RdOff += NumBytesRem;
      }
      pRing->RdOff = RdOff;
    }
  }
  return NumBytesRead;
}

int SEGGER_RTT_HasKey(void) {
  SEGGER_RTT_BUFFER_DOWN* pRing;
  pRing = &_SEGGER_RTT.aDown[0];
  return (pRing->WrOff != pRing->RdOff);
}

int SEGGER_RTT_GetKey(void) {
  char c;
  if (SEGGER_RTT_Read(0, &c, 1) == 1) {
    return (int)(unsigned char)c;
  }
  return -1;
}

int SEGGER_RTT_WaitKey(void) {
  int r;
  do {
    r = SEGGER_RTT_GetKey();
  } while (r < 0);
  return r;
}

int SEGGER_RTT_ConfigUpBuffer(unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags) {
  SEGGER_RTT_BUFFER_UP* pRing;
  
  if (BufferIndex >= SEGGER_RTT_MAX_NUM_UP_BUFFERS) {
    return -1;
  }
  pRing = &_SEGGER_RTT.aUp[BufferIndex];
  SEGGER_RTT_LOCK();
  if (BufferIndex > 0) {
    pRing->sName = sName;
    pRing->pBuffer = (char*)pBuffer;
    pRing->SizeOfBuffer = BufferSize;
    pRing->RdOff = 0;
    pRing->WrOff = 0;
  }
  pRing->Flags = Flags;
  SEGGER_RTT_UNLOCK();
  return 0;
}

int SEGGER_RTT_ConfigDownBuffer(unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags) {
  SEGGER_RTT_BUFFER_DOWN* pRing;
  
  if (BufferIndex >= SEGGER_RTT_MAX_NUM_DOWN_BUFFERS) {
    return -1;
  }
  pRing = &_SEGGER_RTT.aDown[BufferIndex];
  SEGGER_RTT_LOCK();
  if (BufferIndex > 0) {
    pRing->sName = sName;
    pRing->pBuffer = (char*)pBuffer;
    pRing->SizeOfBuffer = BufferSize;
    pRing->WrOff = 0;
    pRing->RdOff = 0;
  }
  pRing->Flags = Flags;
  SEGGER_RTT_UNLOCK();
  return 0;
}

static unsigned char _ActiveTerminal = 0;

int SEGGER_RTT_SetTerminal(unsigned char TerminalId) {
  // Simple terminal switching - just store the active terminal
  // This is used to prefix output with terminal switching commands
  _ActiveTerminal = TerminalId;
  return 0;
}

int SEGGER_RTT_TerminalOut(unsigned char TerminalId, const char* s) {
  // Write string to the specified virtual terminal on channel 0
  // Virtual terminals use special prefix bytes to switch terminals
  unsigned char acSwitch[2];
  unsigned Len;
  
  if (!s) {
    return -1;
  }
  
  Len = strlen(s);
  if (Len == 0) {
    return 0;
  }
  
  SEGGER_RTT_LOCK();
  
  // If switching to a different terminal, send terminal switch command
  if (TerminalId != _ActiveTerminal) {
    acSwitch[0] = 0xFF;  // Terminal switch escape sequence
    acSwitch[1] = (unsigned char)('0' + TerminalId);  // Terminal ID as ASCII character
    SEGGER_RTT_Write(0, acSwitch, 2);
    _ActiveTerminal = TerminalId;
  }
  
  // Write the actual string
  SEGGER_RTT_Write(0, s, Len);
  
  SEGGER_RTT_UNLOCK();
  return (int)Len;
}
