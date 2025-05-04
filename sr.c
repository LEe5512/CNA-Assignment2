#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Selective Repeat protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added Selective Repeat implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12     /* sequence space should be at least 2*WINDOWSIZE */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for (i = 0; i < 20; i++)
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[SEQSPACE];  /* array for storing packets waiting for ACK */
static bool acked[SEQSPACE];         /* Record which packets have been ACKed */
static bool used[SEQSPACE];          /* Mark valid packets */
static int base;                     /* Minimum window number */
static int nextseqnum;               /* Nextseqnum need to be sent */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* Check if window is not full */
  if ((nextseqnum - base + SEQSPACE) % SEQSPACE < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* Store packet in buffer using direct indexing */
    buffer[nextseqnum] = sendpkt;
    used[nextseqnum] = true;
    acked[nextseqnum] = false;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer if first packet in window */
    if (base == nextseqnum)
      starttimer(A, RTT);

    /* Increment sequence number */
    nextseqnum = (nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked, window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data. */
void A_input(struct pkt packet)
{
  int ack = packet.acknum;

  /* if received ACK is not corrupted and is for a packet we sent */
  if (!IsCorrupted(packet) && used[ack]) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", ack);
    total_ACKs_received++;

    /* If not already acknowledged */
    if (!acked[ack]) {
      if (TRACE > 0)
        printf("----A: ACK %d is not a duplicate\n", ack);
      new_ACKs++;
      acked[ack] = true;

      /* If this is the base packet, slide window */
      if (ack == base) {
        /* Stop the timer first */
        stoptimer(A);

        /* Slide window past consecutive ACKed packets */
        while (acked[base]) {
          used[base] = false;
          acked[base] = false;
          base = (base + 1) % SEQSPACE;
        }

        /* Restart timer only if there are still packets in the window */
        if (base != nextseqnum) {
          starttimer(A, RTT);
        }
      }
    }
    else if (TRACE > 0) {
      printf("----A: duplicate ACK received, do nothing!\n");
    }
  }
  else if (TRACE > 0) {
    printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  /* Only retransmit the base packet */
  if (used[base] && !acked[base]) {
    if (TRACE > 0)
      printf("---A: resending packet %d\n", buffer[base].seqnum);
    tolayer3(A, buffer[base]);
    packets_resent++;
  }

  /* Always restart timer */
  starttimer(A, RTT);
}


/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;

  /* initialize A's window, buffer and sequence number */
  base = 0;
  nextseqnum = 0;  /* A starts with seq num 0, do not change this */

  /* Initialize arrays */
  for (i = 0; i < SEQSPACE; i++) {
    acked[i] = false;
    used[i] = false;
  }
}


/********* Receiver (B) variables and procedures ************/

static struct pkt recv_buffer[SEQSPACE];  /* buffer for out-of-order packets */
static bool received[SEQSPACE];          /* track which packets are in buffer */
static int expected_base;                /* the sequence number expected next by the receiver */
static int B_nextseqnum;                 /* the sequence number for the next packets sent by B */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int seq = packet.seqnum;
  int window_end = (expected_base + WINDOWSIZE) % SEQSPACE;
  bool in_window;

  /* Check if the packet is within the receive window */
  if (expected_base <= window_end) {
    in_window = (seq >= expected_base && seq < window_end);
  } else {
    in_window = (seq >= expected_base || seq < window_end);
  }

  /* if packet is not corrupted */
  if (!IsCorrupted(packet)) {
    packets_received++;

    if (in_window) {
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", seq);

      /* If not already received */
      if (!received[seq]) {
        /* Store packet */
        recv_buffer[seq] = packet;
        received[seq] = true;

        /* If this is the expected packet, deliver consecutive packets */
        if (seq == expected_base) {
          while (received[expected_base]) {
            tolayer5(B, recv_buffer[expected_base].payload);
            received[expected_base] = false;
            expected_base = (expected_base + 1) % SEQSPACE;
          }
        }
      }
    } else {
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", seq);
    }

    /* Always send ACK for correctly received packet */
    sendpkt.acknum = seq;
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2;

    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = '0';

    sendpkt.checksum = ComputeChecksum(sendpkt);
    tolayer3(B, sendpkt);
  } else {
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");

    /* Send ACK for the last correctly received packet */
    /* We'll just send an ACK with an invalid acknum for corrupted packets */
    sendpkt.acknum = (expected_base == 0) ? SEQSPACE - 1 : expected_base - 1;
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2;

    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = '0';

    sendpkt.checksum = ComputeChecksum(sendpkt);
    tolayer3(B, sendpkt);
  }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;

  expected_base = 0;
  B_nextseqnum = 1;

  /* Initialize received array */
  for (i = 0; i < SEQSPACE; i++) {
    received[i] = false;
  }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}