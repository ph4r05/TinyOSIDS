/* $Id: QueueC.nc,v 1.7 2009/06/25 18:37:24 scipio Exp $ */
/*
 * Copyright (c) 2006 Stanford University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Stanford University nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL STANFORD
 * UNIVERSITY OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  A general FIFO queue component, whose queue has a bounded size.
 * 
 *  Modified to be used with pointers - object is pushed to this queue and stored in it.
 *  This implementation is bounded on fixed structure size
 *
 *  @author Philip Levis
 *  @author Geoffrey Mainland
 *  @date   $Date: 2009/06/25 18:37:24 $
 */

   
generic module ObjQueueC(typedef queue_t, uint8_t QUEUE_SIZE) {
  provides interface ObjQueue<queue_t> as Queue;
}

implementation {

  queue_t ONE_NOK queue[QUEUE_SIZE];
  uint8_t head = 0;
  uint8_t tail = 0;
  uint8_t size = 0;
  
  command bool Queue.empty() {
    return size == 0;
  }

  command uint8_t Queue.size() {
    return size;
  }

  command uint8_t Queue.maxSize() {
    return QUEUE_SIZE;
  }

  command queue_t * Queue.head() {
    return &(queue[head]);
  }
  
  command queue_t * Queue.dequeue() {
    queue_t * t = call Queue.head();
    dbg("QueueC", "%s: size is %hhu\n", __FUNCTION__, size);
    if (!call Queue.empty()) {
      head++;
      if (head == QUEUE_SIZE) head = 0;
      size--;
    }
    return t;
  }

  command error_t Queue.enqueue(queue_t * newVal) {
    if (call Queue.size() < call Queue.maxSize()) {
      dbg("QueueC", "%s: size is %hhu\n", __FUNCTION__, size);
      
      // copy whole object on correct place
      memcpy((void*) &(queue[tail]), (void*)newVal, sizeof(queue_t));

      tail++;
      if (tail == QUEUE_SIZE) tail = 0;
      size++;
      
      return SUCCESS;
    }
    else {
      return FAIL;
    }
  }
  
  command queue_t * Queue.element(uint8_t idx) {
    idx += head;
    if (idx >= QUEUE_SIZE) {
      idx -= QUEUE_SIZE;
    }
    return &(queue[idx]);
  }  

}
