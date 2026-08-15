/**
  ******************************************************************************
  * @file    test_packet_queue.c
  * @brief   Module contract suite for Core/Src/packet_queue.c (refactor stage 1)
  ******************************************************************************
  * This is a CONTRACT suite, not a findings archive: it documents what the
  * extracted PacketQueue ADT promises, against the real linked module (not a
  * source scan). The Push path is compiled out of flight builds
  * (ENABLE_GNSS_DETAIL_PACKET=0); this target defines it to 1 so the full
  * ADT is exercised (see Makefile, pq target).
  *
  * Covered: init clears; push to full then push again (rejected, no
  * corruption); peek does not consume; pop consumes in FIFO order;
  * wraparound at the head/tail boundary; IsEmpty at every transition;
  * push of size == 0 and size == max.
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "packet_queue.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond) do { \
    g_checks++; \
    if (!(cond)) { \
        g_failures++; \
        printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

/* Fill entry i with a recognizable, per-index pattern. */
static void make_packet(uint8_t *buf, uint16_t size, uint8_t seed)
{
    for (uint16_t i = 0; i < size; i++) buf[i] = (uint8_t)(seed + i);
}

int main(void)
{
    PacketQueue_t q;
    PacketQueueEntry_t e;
    uint8_t buf[150];

    /* ---- init clears a garbage-filled queue ---- */
    memset(&q, 0xAA, sizeof(q));
    PacketQueue_Init(&q);
    CHECK(q.count == 0);
    CHECK(q.head == 0);
    CHECK(q.tail == 0);
    CHECK(PacketQueue_IsEmpty(&q) == true);

    /* ---- empty-queue peek/pop are rejected ---- */
    CHECK(PacketQueue_Peek(&q, &e) == false);
    CHECK(PacketQueue_Pop(&q, &e) == false);

    /* ---- single push: not empty, count tracks ---- */
    make_packet(buf, 10, 0x10);
    CHECK(PacketQueue_Push(&q, buf, 10, 42) == true);
    CHECK(PacketQueue_IsEmpty(&q) == false);
    CHECK(q.count == 1);

    /* ---- peek does not consume (twice, identical) ---- */
    CHECK(PacketQueue_Peek(&q, &e) == true);
    CHECK(e.size == 10 && e.port == 42);
    CHECK(q.count == 1);
    CHECK(PacketQueue_Peek(&q, &e) == true && e.port == 42);
    CHECK(q.count == 1);

    /* ---- pop returns the same data and empties the queue ---- */
    CHECK(PacketQueue_Pop(&q, &e) == true);
    CHECK(e.size == 10 && e.port == 42 && memcmp(e.buffer, buf, 10) == 0);
    CHECK(q.count == 0);
    CHECK(PacketQueue_IsEmpty(&q) == true);

    /* ---- fill to full; the (SIZE+1)th push is rejected without corruption ----
     * Re-init first: head/tail are NOT reset by draining (circular pointers
     * only return to 0 by wraparound), so pointer assertions need a known
     * zeroed start. */
    PacketQueue_Init(&q);
    for (uint8_t i = 0; i < PACKET_QUEUE_SIZE; i++) {
        make_packet(buf, 4, (uint8_t)(0x20 + i));
        CHECK(PacketQueue_Push(&q, buf, 4, i) == true);
    }
    CHECK(q.count == PACKET_QUEUE_SIZE);
    CHECK(q.head == 0);            /* wrapped after exactly SIZE pushes */
    CHECK(q.tail == 0);
    make_packet(buf, 4, 0xEE);
    CHECK(PacketQueue_Push(&q, buf, 4, 0xEE) == false);   /* full: rejected */
    CHECK(q.count == PACKET_QUEUE_SIZE);
    CHECK(q.head == 0 && q.tail == 0);                    /* no corruption */

    /* ---- pop all in FIFO order ---- */
    bool order_ok = true, all_popped = true;
    for (uint8_t i = 0; i < PACKET_QUEUE_SIZE; i++) {
        if (PacketQueue_Pop(&q, &e) != true) all_popped = false;
        if (e.port != i || e.buffer[0] != (uint8_t)(0x20 + i)) order_ok = false;
    }
    CHECK(all_popped);
    CHECK(order_ok);
    CHECK(PacketQueue_IsEmpty(&q) == true);

    /* ---- wraparound: push 6, pop 4, push 6 (head wraps mid-fill), drain ---- */
    for (uint8_t i = 0; i < 6; i++) { make_packet(buf, 2, i); PacketQueue_Push(&q, buf, 2, i); }
    for (uint8_t i = 0; i < 4; i++) { PacketQueue_Pop(&q, &e); }
    for (uint8_t i = 6; i < 12; i++) { make_packet(buf, 2, i); PacketQueue_Push(&q, buf, 2, i); }
    CHECK(q.count == PACKET_QUEUE_SIZE);
    CHECK(q.head == 4 && q.tail == 4);                    /* wrapped pointers meet */
    bool wrap_order = true;
    for (uint8_t i = 4; i < 12; i++) {
        if (PacketQueue_Pop(&q, &e) != true || e.port != i) wrap_order = false;
    }
    CHECK(wrap_order);
    CHECK(PacketQueue_IsEmpty(&q) == true);

    /* ---- size boundaries: 0 accepted, max (150) accepted, max+1 rejected ---- */
    CHECK(PacketQueue_Push(&q, buf, 0, 7) == true);
    CHECK(PacketQueue_Pop(&q, &e) == true && e.size == 0);
    make_packet(buf, (uint16_t)sizeof(e.buffer), 0x55);
    CHECK(PacketQueue_Push(&q, buf, (uint16_t)sizeof(e.buffer), 8) == true);
    CHECK(PacketQueue_Pop(&q, &e) == true && e.size == sizeof(e.buffer) &&
          memcmp(e.buffer, buf, sizeof(e.buffer)) == 0);
    CHECK(PacketQueue_Push(&q, buf, (uint16_t)(sizeof(e.buffer) + 1U), 9) == false);
    CHECK(PacketQueue_IsEmpty(&q) == true);

    printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures != 0;
}
