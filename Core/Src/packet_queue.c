/**
 ******************************************************************************
 * @file    packet_queue.c
 * @brief   Circular packet queue for deferred LoRaWAN transmission
 ******************************************************************************
 * Function bodies moved verbatim from LoRaWAN/App/lora_app.c (refactor
 * stage 1; mechanical move, no behaviour change). Only the `static`
 * storage-class was removed so the ADT is linkable and host-testable.
 ******************************************************************************
 */

#include "packet_queue.h"
#include <string.h>

/**
 * @brief  Initialize packet queue
 * @param  queue: Pointer to queue structure
 * @retval None
 */
void PacketQueue_Init(PacketQueue_t *queue) {
  memset(queue, 0, sizeof(PacketQueue_t));
}

/* FR-19 (#100) gate removed (A-005/#79): version-report is a second producer. */
/**
 * @brief  Push packet to queue
 * @param  queue: Pointer to queue structure
 * @param  data: Packet data
 * @param  size: Packet size
 * @param  port: LoRaWAN port
 * @retval true if successful, false if queue full
 */
bool PacketQueue_Push(PacketQueue_t *queue, const uint8_t *data, uint16_t size, uint8_t port) {
  if (queue->count >= PACKET_QUEUE_SIZE || size > sizeof(queue->entries[0].buffer))
    return false;

  PacketQueueEntry_t *entry = &queue->entries[queue->head];
  memcpy(entry->buffer, data, size);
  entry->size = size;
  entry->port = port;
  entry->valid = true;

  queue->head = (queue->head + 1) % PACKET_QUEUE_SIZE;
  queue->count++;

  return true;
}

/**
 * @brief  Peek at the head packet WITHOUT removing it (STAB-P3#7, #243)
 * @param  queue: Pointer to queue structure
 * @param  entry: Destination for peeked entry
 * @retval true if successful, false if queue empty
 */
bool PacketQueue_Peek(PacketQueue_t *queue, PacketQueueEntry_t *entry) {
  if (queue->count == 0)
    return false;

  *entry = queue->entries[queue->tail];
  return true;
}

/**
 * @brief  Pop packet from queue
 * @param  queue: Pointer to queue structure
 * @param  entry: Destination for popped entry
 * @retval true if successful, false if queue empty
 */
bool PacketQueue_Pop(PacketQueue_t *queue, PacketQueueEntry_t *entry) {
  if (queue->count == 0)
    return false;

  *entry = queue->entries[queue->tail];
  queue->entries[queue->tail].valid = false;

  queue->tail = (queue->tail + 1) % PACKET_QUEUE_SIZE;
  queue->count--;

  return true;
}

/**
 * @brief  Check if queue is empty
 * @param  queue: Pointer to queue structure
 * @retval true if empty
 */
bool PacketQueue_IsEmpty(PacketQueue_t *queue) {
  return (queue->count == 0);
}

/* FR-19 (#100): PacketQueue_Count deleted — its only callers were log lines;
 * use g_packet_queue.count directly. */
