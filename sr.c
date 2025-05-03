#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "gbn.h"

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
#define SEQSPACE 12      /* for sr.c we need 2*window_size as seqspace */
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
  for ( i=0; i<20; i++ )
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

static struct {
    struct pkt packet;  /* packet stored in the buffer */
    int acked;          /* to define the packet is confirmed or not */
} A_buffer[WINDOWSIZE]; /* send window buffer */

static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static int timer_seq;                  /* sequence number of the packet that is being timed */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ )
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */
    windowlast = (windowlast + 1) % WINDOWSIZE;
    A_buffer[windowlast].packet = sendpkt;
    A_buffer[windowlast].acked = 0;  /* mark as not acknowledged */

    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer if first packet in window */
    if (windowcount == 1) {
      starttimer(A,RTT);
      timer_seq = sendpkt.seqnum;
    }

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked,  window is full */
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
  int i;
  int next_timeout_idx = -1;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* search for the packet with this sequence number in our window */
    for (i = 0; i < windowcount; i++) {
      int pos = (windowfirst + i) % WINDOWSIZE;

      /* if we find the packet and it's not already acknowledged */
      if (A_buffer[pos].packet.seqnum == packet.acknum && !A_buffer[pos].acked) {
        /* mark as acknowledged */
        A_buffer[pos].acked = 1;

        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        new_ACKs++;

        break;
      }
    }

    /* if we acknowledged the packet that our timer is for, we need to reset the timer */
    if (packet.acknum == timer_seq) {
      stoptimer(A);

      /* find the next unacknowledged packet */
      for (i = 0; i < windowcount; i++) {
        int pos = (windowfirst + i) % WINDOWSIZE;
        if (!A_buffer[pos].acked) {
          next_timeout_idx = pos;
          break;
        }
      }

      /* if we found another unacknowledged packet, set the timer for it */
      if (next_timeout_idx != -1) {
        starttimer(A, RTT);
        timer_seq = A_buffer[next_timeout_idx].packet.seqnum;
      }
    }

    /* try to slide the window */
    while (windowcount > 0 && A_buffer[windowfirst].acked) {
      windowfirst = (windowfirst + 1) % WINDOWSIZE;
      windowcount--;
    }

    /* if the window is now empty, stop the timer */
    if (windowcount == 0) {
      stoptimer(A);
    }
  }
  else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i, found = 0;
  int next_timeout_idx = -1;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  /* find the packet our timer is for and resend it */
  for (i = 0; i < windowcount; i++) {
    int pos = (windowfirst + i) % WINDOWSIZE;

    /* if this is the packet we're timing and it's not acknowledged */
    if (A_buffer[pos].packet.seqnum == timer_seq && !A_buffer[pos].acked) {
      /* resend the packet */
      tolayer3(A, A_buffer[pos].packet);
      packets_resent++;
      found = 1;

      /* restart the timer for this packet */
      starttimer(A, RTT);
      break;
    }
  }

  /* if we didn't find the timed packet (it might have been acknowledged),
     find the next unacknowledged packet */
  if (!found) {
    for (i = 0; i < windowcount; i++) {
      int pos = (windowfirst + i) % WINDOWSIZE;
      if (!A_buffer[pos].acked) {
        next_timeout_idx = pos;
        break;
      }
    }

    /* if we found another unacknowledged packet, set the timer for it */
    if (next_timeout_idx != -1) {
      timer_seq = A_buffer[next_timeout_idx].packet.seqnum;
      starttimer(A, RTT);
    }
  }
}



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.
		     new packets  are placed in winlast + 1
		     so initially this is set to -1
		   */
  windowcount = 0;
  timer_seq = -1; /* because no packet is timed at beginning */
  /* initialize buffer */
  for (i = 0; i < WINDOWSIZE; i++) {
      A_buffer[i].acked = 0;
  }
}



/********* Receiver (B)  variables and procedures ************/

static struct {
    struct pkt packet;   /* packet stored in the buffer */
    int received;       /* whether the packet is received: 0=not received, 1=received */
} B_buffer[WINDOWSIZE];   /* receive window buffer */

static int B_windowfirst;            /* first sequence number in the receiver's window */
static int B_nextseqnum;             /* the sequence number for the next packets sent by B */


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i, seqnum, offset;

  seqnum = packet.seqnum;

  /* if the packet is not corrupted */
  if (!IsCorrupted(packet)) {
    /* calculate offset from the beginning of the receive window */
    offset = (seqnum - B_windowfirst + SEQSPACE) % SEQSPACE;

    /* check if the packet is within the receive window range */
    if (offset < WINDOWSIZE) {
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", seqnum);

      /* store the packet */
      B_buffer[offset].packet = packet;
      B_buffer[offset].received = 1;

      /* if this is the leftmost packet in the window, try to deliver to upper layer */
      while (B_buffer[0].received) {
        /* deliver to application layer */
        tolayer5(B, B_buffer[0].packet.payload);
        packets_received++;

        /* slide the window */
        for (i = 0; i < WINDOWSIZE - 1; i++) {
          B_buffer[i] = B_buffer[i + 1];
        }

        /* clear the rightmost position in the window */
        B_buffer[WINDOWSIZE - 1].received = 0;

        /* update the first sequence number of the window */
        B_windowfirst = (B_windowfirst + 1) % SEQSPACE;
      }
    }

    /* send ACK regardless of whether the packet is in-window or not */
    sendpkt.acknum = seqnum;
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2;

    /* fill payload with zeros */
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = '0';

    /* compute checksum */
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* send the ACK packet */
    tolayer3(B, sendpkt);
  }
  else {
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
  }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;

  /* initialize receiver window */
  B_windowfirst = 0;
  B_nextseqnum = 1;

  /* initialize receiver buffer */
  for (i = 0; i < WINDOWSIZE; i++) {
      B_buffer[i].received = 0;
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