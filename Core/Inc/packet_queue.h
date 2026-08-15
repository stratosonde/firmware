/**
  ******************************************************************************
  * @file    packet_queue.h
  * @brief   Circular packet queue for deferred LoRaWAN transmission
  ******************************************************************************
  * Pure abstract data type extracted verbatim from LoRaWAN/App/lora_app.c
  * (refactor stage 1; mechanical move, no behaviour change). No hardware, no
  * file-scope mutable state: the caller owns the PacketQueue_t instance.
  ******************************************************************************
  */

#ifndef PACKET_QUEUE_H
#define PACKET_QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "payload_format.h"  /* ENABLE_GNSS_DETAIL_PACKET */

/**
  * @brief Packet queue entry for deferred LoRaWAN transmission
  */
typedef struct
{
  uint8_t buffer[150];           // Packet data buffer
  uint16_t size;                 // Actual packet size
  uint8_t port;                  // LoRaWAN port number
  bool valid;                    // Entry is occupied
} PacketQueueEntry_t;

/**
  * @brief Simple circular packet queue
  */
#define PACKET_QUEUE_SIZE 8      // Max packets in queue
typedef struct
{
  PacketQueueEntry_t entries[PACKET_QUEUE_SIZE];
  uint8_t head;                  // Write position
  uint8_t tail;                  // Read position
  uint8_t count;                 // Current number of packets
} PacketQueue_t;

/**
  * @brief  Initialize packet queue
  * @param  queue: Pointer to queue structure
  * @retval None
  */
void PacketQueue_Init(PacketQueue_t *queue);

#if ENABLE_GNSS_DETAIL_PACKET  /* FR-19 (#100): sole producer is the debug packet */
/**
  * @brief  Push packet to queue
  * @param  queue: Pointer to queue structure
  * @param  data: Packet data
  * @param  size: Packet size
  * @param  port: LoRaWAN port
  * @retval true if successful, false if queue full
  */
bool PacketQueue_Push(PacketQueue_t *queue, const uint8_t *data, uint16_t size, uint8_t port);
#endif  /* ENABLE_GNSS_DETAIL_PACKET */

/**
  * @brief  Peek at the head packet WITHOUT removing it (STAB-P3#7, #243)
  * @param  queue: Pointer to queue structure
  * @param  entry: Destination for peeked entry
  * @retval true if successful, false if queue empty
  */
bool PacketQueue_Peek(PacketQueue_t *queue, PacketQueueEntry_t *entry);

/**
  * @brief  Pop packet from queue
  * @param  queue: Pointer to queue structure
  * @param  entry: Destination for popped entry
  * @retval true if successful, false if queue empty
  */
bool PacketQueue_Pop(PacketQueue_t *queue, PacketQueueEntry_t *entry);

/**
  * @brief  Check if queue is empty
  * @param  queue: Pointer to queue structure
  * @retval true if empty
  */
bool PacketQueue_IsEmpty(PacketQueue_t *queue);

#ifdef __cplusplus
}
#endif

#endif /* PACKET_QUEUE_H */
