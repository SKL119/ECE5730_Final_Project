/**
 * Copyright (c) 2022 Andrew McDonnell
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * UDP sned/receive Adapted from:
 ** Copyright (c) 2016 Stephan Linz <linz@li-pro.net>, Li-Pro.Net
 * All rights reserved.
 *
 * Based on examples provided by
 * Iwan Budi Kusnanto <ibk@labhijau.net> (https://gist.github.com/iwanbk/1399729)
 * Juri Haberland <juri@sapienti-sat.org> (https://lists.gnu.org/archive/html/lwip-users/2007-06/msg00078.html)
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of and a contribution to the lwIP TCP/IP stack.
 * Credits go to Adam Dunkels (and the current maintainers) of this software.
 * Stephan Linz rewrote this file to get a basic echo example.
 * =============================================================
 * UDP send/recv code is from :
 * Pico examples  
 * https://github.com/raspberrypi/pico-examples/tree/master/pico_w/wifi/udp_beacon
 * lwip contrib apps 
 * https://github.com/lwip-tcpip/lwip/tree/master/contrib/apps
 * UDP send/recv on Windows is from:
 * Microsoft 
 * https://apps.microsoft.com/store/detail/udp-senderreciever/9NBLGGH52BT0?hl=en-us&gl=us
 * a bare-bones packet whacker
 * =============================================================
 * One picoW is an AP, the other is a station
 * Threads:
 * Core0:
 * -- udp send -- core1 prompt for, then send, a string
 * -- udp recv -- core1 prints the received string + time and packet count
 * -- packet count is shared over the network between the two systems 
 * -- LWIP receive callback routine
 * Core1:
 * -- blink cyw43 LED
 * -- serial for sneding a string
 * Main:
 * checks for a 'true' on gpio2 and sets AP 
 * assigns adressses, which you may want to modify
 */

#include <string.h>
#include <stdlib.h>
#include <pico/multicore.h>
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "stdio.h"
#include "math.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/dns.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "dhcpserver/dhcpserver.h"

#include "vga_graphics.h"
#include "mpu6050.h"

#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
//#include "pt_cornell_rp2040_v1.h"

#define GPIO_BUTTON1      15
#define GPIO_BUTTON2      21

// ======================================
// set automatic setup on if defined
// must use this in this program
#define auto_setup

// udp constants
#define UDP_PORT 4444
#define UDP_MSG_LEN_MAX 1400 // max length without fragments
// should resolve to a actual addr after 'main' executes
char udp_target_pico[20] ; 

// =======================================
// protothreads and thread communication
#include "pt_cornell_rp2040_v1_3.h"
char recv_data[UDP_MSG_LEN_MAX];
char send_data[UDP_MSG_LEN_MAX];


fix15 angles[3];

// character array
char restarttext[20];
char easytext[20];
char midtext[20];
char hardtext[20];
char level1text[20];
char level2text[20];
char level3text[20];
char level4text[20];
char youwintext[20];
char youlosetext[20];
char nextleveltext[20];
char timetext[20];

#define abs(a) ((a>0) ? a:-a)

// maze box parameters
#define plane_x0         (178)
#define plane_y0         (96)
#define plane_width      (384) // 32 * 11 + 4 * 10 - 8
#define plane_height     (276) // 32 * 8 + 4 * 7 - 8
#define plane_x1         (plane_x0 + plane_width)
#define plane_y1         (plane_y0 + plane_height)
#define wall_thickness   (4)
#define quad_size        (8)

#define box_x0           (plane_x0 - wall_thickness)
#define box_y0           (plane_y0 - wall_thickness)
#define box_x1           (plane_x1 + quad_size)
#define box_y1           (plane_y1 + quad_size)
#define box_width        (plane_width + wall_thickness + quad_size)
#define box_height       (plane_height+ wall_thickness + quad_size)

#define endpoint_size    (quad_size << 1)

#define endpoint_1_x0    (plane_x1 - endpoint_size)
#define endpoint_1_y0    (plane_y1 - endpoint_size)

#define trap_1_x0    (plane_x0 + 144)
#define trap_1_y0    (plane_y0 + 100)
#define trap_2_x0    (box_x1 - endpoint_size)
#define trap_2_y0    (plane_y0 + endpoint_size)

// quad movement parameters
#define  freq             40
#define  dt               float2fix15(1.0/freq)

#define  pioveroneeighty  float2fix15(M_PI / 180.0)

#define  g1               int2fix15(500) // g = 10 m/s^2 => 1000 cm / s^2; 1 cm = 16 pixels; 62.5 pixels / s^2
#define  g2               int2fix15(1000)
#define  g3               int2fix15(1600)
#define  d                float2fix15(0.875)

#define  v_threshold      (int2fix15(1))

#define wall_block_length (36)

volatile fix15 g  = g1;
volatile fix15 av = int2fix15(0);
volatile fix15 ah = int2fix15(0);
volatile fix15 vv = int2fix15(0);
volatile fix15 vh = int2fix15(0);
volatile fix15 pv = int2fix15(plane_x0 + wall_thickness);
volatile fix15 ph = int2fix15(plane_y0 + wall_thickness);
volatile fix15 py = int2fix15(wall_thickness);
volatile fix15 px = int2fix15(wall_thickness);
volatile fix15 cos_angle_v, cos_angle_h;

// 0 - restart
// 1 - easy g = 500
// 2 - mid  g = 1000
// 3 - hard g = 1600
// 4 - level 1: empty plane
// 5 - level 2: maze
// 6 - level 3: maze + trap
// 7 - level 4: maze + trap + timer

int state = 0;
int level = 0;
int button1_pressed = 0;
int button1_previous = 0;
int button2_pressed = 0;
int button2_previous = 0;
int restart = 0;
int win = 0;
int win_previous = 0;
int win_state = 0;

int lose = 0;
int lose_previous = 0;

int row = 1;
int col = 1;
volatile fix15 block_x0 = int2fix15(plane_x0);
volatile fix15 block_y0 = int2fix15(plane_y0);

float level4_time = 20.0; 

char vWalls[9][12] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
    {1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1}, 
    {1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1}, 
    {1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1}, 
    {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1}, 
    {1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1}, 
    {1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1}, 
    {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1}, 
};
char hWalls[9][12] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 
    {0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0}, 
    {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}, 
    {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, 
    {0, 1, 1, 1, 1, 0, 0, 1, 0, 1, 0, 0}, 
    {0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0}, 
    {0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0}, 
    {0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0}, 
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, 
};
char corners[9][12] = {
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {0, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0}, 
    {0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0}, 
    {0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0}, 
    {0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0}, 
    {0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0}, 
    {0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0}, 
    {0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0}, 
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
};

void drawMaze() {
    for(int i = 1; i < 9; i++){
        for(int j = 1; j < 11; j++){
            if (vWalls[i][j] == 1) {
                fillRect(plane_x0 - 4 + j *wall_block_length, plane_y0 - 36 + i*wall_block_length, wall_thickness, wall_block_length, WHITE) ;
            }
        }
    }

    for(int i = 1; i < 8; i++){
        for(int j = 1; j < 12; j++){
            if (hWalls[i][j] == 1) {
                fillRect(plane_x0 - 36 + j*wall_block_length, plane_y0 - 4 + i*wall_block_length, wall_block_length, wall_thickness, WHITE) ;
            }
        }
    }

    for(int i = 1; i < 8; i++){
        for(int j = 1; j < 11; j++){
            if (corners[i][j] == 1) {
                fillRect(plane_x0 - 4 + j*wall_block_length, plane_y0 - 4 + i*wall_block_length, wall_thickness, wall_thickness, WHITE) ;
            }
        }
    }
}

// led blink
int blink_time ;

// count the number of packet sent
int packet_count ;
// the send time
int64_t raw_time ;

// interthread communicaition
// signal threads for sned/recv data
struct pt_sem new_udp_recv_s, new_udp_send_s ;

//==================================================
// UDP async receive callback setup
// NOTE that udpecho_raw_recv is triggered by a signal
// directly from the LWIP package -- not from your code
// this callback just copies out the packet string
// and sets a "new data" semaphore
// This runs in an ISR -- KEEP IT SHORT!!!
// You SHOUND NOT need to change this

// ==================================================
#if LWIP_UDP
// >>>don't change anything in the LWIP_UDP section, without careful understanding<<<

static struct udp_pcb *udpecho_raw_pcb;
//struct pbuf *p ;

//static void
void __not_in_flash_func(udpecho_raw_recv)(void *arg, struct udp_pcb *upcb, struct pbuf *p,
                 const ip_addr_t *addr, u16_t port)
{
  LWIP_UNUSED_ARG(arg);
  if (p != NULL) {
    //printf("p payload in call back: = %s\n", p->payload);
    memcpy(recv_data, p->payload, UDP_MSG_LEN_MAX);
    /* free the pbuf */ 
    pbuf_free(p);
    // can signal from an ISR -- BUT NEVER wait in an ISR
    PT_SEM_SIGNAL(pt, &new_udp_recv_s) ;
  }
  else printf("NULL pt in callback");
}

// ===================================
// Define the recv callback to LWIP
// dont change this
// ===================================
void 
udpecho_raw_init(void)
{
  struct pbuf *p ; // OMVED
  udpecho_raw_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  p = pbuf_alloc(PBUF_TRANSPORT, UDP_MSG_LEN_MAX+1, PBUF_POOL);

  if (udpecho_raw_pcb != NULL) {
    err_t err;
    // netif_ip4_addr returns the picow ip address
    err = udp_bind(udpecho_raw_pcb, netif_ip4_addr(netif_list), UDP_PORT); //DHCP addr

    if (err == ERR_OK) {
      udp_recv(udpecho_raw_pcb, udpecho_raw_recv, NULL);
      //printf("Set up recv callback\n");
    } else {
      printf("bind error");
    }
  } else {
    printf("udpecho_raw_pcb error");
  }
}

#endif /* LWIP_UDP */
// end recv setup
// don't change anything in the LWIP_UDP section, without careful understanding
// ==================================================

// This timer ISR is called on core 0
bool maze_game() {
    if (win || lose) {
        return true;
    }

    if (restart) {
        if (level == 3) {
            level4_time = 20.0;
        }
        row = 1;
        col = 1;
        block_x0 = int2fix15(plane_x0);
        block_y0 = int2fix15(plane_y0);
        vv = int2fix15(0);
        vh = int2fix15(0);
        //fillRect(fix2int15(ph), fix2int15(pv), quad_size, quad_size, BLACK) ;
        pv = int2fix15(plane_y0 + wall_thickness);
        ph = int2fix15(plane_x0 + wall_thickness);
        fillRect(fix2int15(ph), fix2int15(pv), quad_size, quad_size, CYAN) ;
        restart = 0;
        return true;
    }

    // ============================ Update a,v,p ============================
    // update acceleration 
    // av = -(multfix15(multfix15(float2fix15(cos(angle_v_rad)), g), difficulty1));
    // ah = -(multfix15(multfix15(float2fix15(cos(angle_h_rad)), g), difficulty1));
    fix15 g0 = g;
    av = -(multfix15(cos_angle_v, g0));
    ah = -(multfix15(cos_angle_h, g0));

    // update velocity
    vv = multfix15(vv, d) + multfix15(av, dt);
    vh = multfix15(vh, d) + multfix15(ah, dt);

    if (abs(vv) < v_threshold) {
        vv = int2fix15(0);
    }
    if (abs(vh) < v_threshold) {
        vh = int2fix15(0);
    }
    
    // erase the quad from the old position
    fillRect(fix2int15(ph), fix2int15(pv), quad_size, quad_size, BLACK) ;

    // calculate new position
    pv = pv + multfix15(vv, dt);
    ph = ph + multfix15(vh, dt);

    // check if reach the endpoint, restart
    if (ph > int2fix15(endpoint_1_x0 - (quad_size >> 1))
        && ph < int2fix15(endpoint_1_x0 + endpoint_size - (quad_size >> 1))
        && pv > int2fix15(endpoint_1_y0 - (quad_size >> 1))
        && pv < int2fix15(endpoint_1_y0 + endpoint_size - (quad_size >> 1)))
    {
        pv = int2fix15(plane_y0 + wall_thickness);
        ph = int2fix15(plane_x0 + wall_thickness);
        // restart = 1;
        win = 1;
        win_state = 0;
        return true;
    }

    if (level == 0) {
        // check if hit the boundry
        if (pv < int2fix15(plane_y0)) {
            pv = int2fix15(plane_y0);
            vv = -vv;
        } else if (pv > int2fix15(plane_y1)) {
            pv = int2fix15(plane_y1);
            vv = -vv;
        }

        if (ph < int2fix15(plane_x0)) {
            ph = int2fix15(plane_x0);
            vh = -vh;
        } else if (ph > int2fix15(plane_x1)) {
            ph = int2fix15(plane_x1);
            vh = -vh;
        }

        fillRect(fix2int15(ph), fix2int15(pv), quad_size, quad_size, CYAN) ;
        return true;
    }

    if (level > 1) {
        // check if reach the trap, restart
        if (ph > int2fix15(trap_1_x0 - (quad_size >> 1))
            && ph < int2fix15(trap_1_x0 + endpoint_size - (quad_size >> 1))
            && pv > int2fix15(trap_1_y0 - (quad_size >> 1))
            && pv < int2fix15(trap_1_y0 + endpoint_size - (quad_size >> 1)))
        {
            pv = int2fix15(plane_y0 + wall_thickness);
            ph = int2fix15(plane_x0 + wall_thickness);

            lose = 1;
            return true;
        }

        if (ph > int2fix15(trap_2_x0 - (quad_size >> 1))
            && ph < int2fix15(trap_2_x0 + endpoint_size - (quad_size >> 1))
            && pv > int2fix15(trap_2_y0 - (quad_size >> 1))
            && pv < int2fix15(trap_2_y0 + endpoint_size - (quad_size >> 1)))
        {
            pv = int2fix15(plane_y0 + wall_thickness);
            ph = int2fix15(plane_x0 + wall_thickness);

            lose = 1;
            return true;
        }
    }

    if (level == 3) {
        level4_time -= 0.025;
        if (level4_time <= 0.0) {
            lose = 1;
            return true;
        }
    }

    // check if hit maze walls
    
    px = ph - block_x0;
    py = pv - block_y0;

    // check left wall
    if (vWalls[row][col - 1] == 1 && px < int2fix15(0)) {
        vh = -vh;
        px = int2fix15(0);
    }
    
    // check right wall
    if (vWalls[row][col]) {
        if (px > int2fix15(24)) {
            vh = -vh;
            px = int2fix15(24);
        }
    }

    // check upper wall
    if (hWalls[row - 1][col]) {
        if (py < int2fix15(0)) {
            vv = -vv;
            py = int2fix15(0);
        }
    }

    // check under wall
    if (hWalls[row][col]) {
        if (py > int2fix15(24)) {
            vv = -vv;
            py = int2fix15(24);
        }
    }

    // check upper left corner
    if ((corners[row - 1][col - 1] == 1) && (vWalls[row][col - 1] == 0) && (hWalls[row - 1][col] == 0)) {
        if (px < int2fix15(0) && py < int2fix15(0)) {
            vh = -vh;
            vv = -vv;
            px = int2fix15(0);
            py = int2fix15(0);
        }
    }

    // check upper right corner
    if ((corners[row - 1][col] == 1) && (vWalls[row][col] == 0) && (hWalls[row - 1][col] == 0)) {
        if (px > int2fix15(24) && py < int2fix15(0)) {
            vh = -vh;
            vv = -vv;
            px = int2fix15(24);
            py = int2fix15(0);
        }
    }

    // check under left corner
    if ((corners[row][col - 1] == 1) && (vWalls[row][col - 1] == 0) && (hWalls[row][col] == 0)) {
        if (px < int2fix15(0) && py > int2fix15(24)) {
            vh = -vh;
            vv = -vv;
            px = int2fix15(0);
            py = int2fix15(24);
        }
    }

    // check under right corner
    if ((corners[row][col] == 1) && (vWalls[row][col] == 0) && (hWalls[row][col] == 0)) {
        if (px > int2fix15(24) && py > int2fix15(24)) {
            vh = -vh;
            vv = -vv;
            px = int2fix15(24);
            py = int2fix15(24);
        }
    }

    if (px < int2fix15(0)) {
        col --;
        px = px + int2fix15(36);
        block_x0 = block_x0 - int2fix15(36);
    } else if (px > int2fix15(36)) {
        col ++;
        px = px - int2fix15(36);
        block_x0 = block_x0 + int2fix15(36);
    }

    if (py < int2fix15(0)) {
        row --;
        py = py + int2fix15(36);
        block_y0 = block_y0 - int2fix15(36);
    } else if (py > int2fix15(36)) {
        row ++;
        py = py - int2fix15(36);
        block_y0 = block_y0 + int2fix15(36);
    }

    ph = px + block_x0;
    pv = py + block_y0;

    // draw the quad at the new position
    fillRect(fix2int15(ph), fix2int15(pv), quad_size, quad_size, CYAN) ;

    return true;
}

// ==================================================
// udp recv processing
// this is where you put your RECEIVE CODE
// receives either strings or arrays
// first byte==0x02 => string
// first byte==0xff => aray
// ==================================================
static PT_THREAD (protothread_udp_recv(struct pt *pt))
{
    PT_BEGIN(pt);
      static fix15 data[5];
      //
      while(1) {
        // wait for new packet
        // signalled by LWIP receive ISR
        PT_SEM_WAIT(pt, &new_udp_recv_s) ;   

        if (recv_data[0] == 0xee) {
          memcpy(data, recv_data, 5*__SIZEOF_INT__) ;
          cos_angle_v = data[1];
          cos_angle_h = data[2];
          button1_pressed = fix2int15(data[3]);
          button2_pressed = fix2int15(data[4]);
          printf("cos_angle_v = %5.2f  cos_angle_h = %5.2f\n\r", fix2float15(cos_angle_v), fix2float15(cos_angle_h) ) ;
          maze_game();
          printf("av = %5.2f  ah = %5.2f\n\r", fix2float15(av), fix2float15(ah) ) ;
          printf("pv = %5.2f  ph = %5.2f  button1_pressed = %d  button2_pressed = %d \n\r", fix2float15(pv), fix2float15(ph), button1_pressed, button2_pressed ) ;
        }
        // NEVER exit while
      } // END WHILE(1)
    PT_END(pt);
} // recv thread

// =================================================
// VGA display thread
// =================================================
// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // Draw the static aspects of the display
    sprintf(restarttext, "Restart") ;
    sprintf(easytext, "Easy") ;
    sprintf(midtext, "Mid") ;
    sprintf(hardtext, "Hard") ;
    sprintf(level1text, "Level 1") ;
    sprintf(level2text, "Level 2") ;
    sprintf(level3text, "Level 3") ;
    sprintf(level4text, "Level 4") ;
    sprintf(youwintext, "You Win ~") ;
    sprintf(youlosetext, "You Lose...") ;
    sprintf(nextleveltext, "Next Level") ;

    setTextSize(2) ;

    setTextColor(MAGENTA);
    setCursor(plane_x0, 60) ;
    writeString(level1text) ;
    setTextColor(GREEN);
    setCursor(plane_x0 + 130, 60) ;
    writeString(easytext) ;


    setTextColor(GREEN);
    setCursor(40, 100) ;
    writeString(restarttext) ;
    setTextColor(WHITE);
    setCursor(40, 130) ;
    writeString(easytext) ;
    setCursor(40, 150) ;
    writeString(midtext) ;
    setCursor(40, 170) ;
    writeString(hardtext) ;
    setCursor(40, 210) ;
    writeString(level1text) ;
    setCursor(40, 230) ;
    writeString(level2text) ;
    setCursor(40, 250) ;
    writeString(level3text) ;
    setCursor(40, 270) ;
    writeString(level4text) ;
    
    fillRect(box_x0, box_y0, box_width, wall_thickness, WHITE) ;
    fillRect(plane_x0, box_y1, box_width, wall_thickness, WHITE) ;

    fillRect(box_x0, plane_y0, wall_thickness, box_height, WHITE) ;
    fillRect(box_x1, box_y0, wall_thickness, box_height, WHITE) ;


    fillRect(endpoint_1_x0, endpoint_1_y0, endpoint_size, endpoint_size, GREEN) ;
    

    while (1) {
        if (button1_pressed == 1 && button1_previous == 0) {
            button1_previous = 1;

            if (win) {
                if (win_state == 0) {
                    win_state = 1;
                    setTextColor(WHITE);
                    setCursor(320, 260) ;
                    writeString(nextleveltext) ;
                    setTextColor(GREEN);
                    setCursor(320, 290) ;
                    writeString(restarttext) ;
                } else {
                    win_state = 0;
                    setTextColor(GREEN);
                    setCursor(320, 260) ;
                    writeString(nextleveltext) ;
                    setTextColor(WHITE);
                    setCursor(320, 290) ;
                    writeString(restarttext) ;
                }
            } else if (lose == 0) {
                state = (state + 1) % 8;
                setTextColor(WHITE);
                if (state == 0) {
                    setCursor(40, 270) ;
                    writeString(level4text) ;
                    setTextColor(GREEN);
                    setCursor(40, 100) ;
                    writeString(restarttext) ;
                } else if (state == 1) {
                    setCursor(40, 100) ;
                    writeString(restarttext) ;
                    setTextColor(GREEN);
                    setCursor(40, 130) ;
                    writeString(easytext) ;
                } else if (state == 2) {
                    setCursor(40, 130) ;
                    writeString(easytext) ;
                    setTextColor(GREEN);
                    setCursor(40, 150) ;
                    writeString(midtext) ;
                } else if (state == 3) {
                    setCursor(40, 150) ;
                    writeString(midtext) ;
                    setTextColor(GREEN);
                    setCursor(40, 170) ;
                    writeString(hardtext) ;
                } else if (state == 4) {
                    setCursor(40, 170) ;
                    writeString(hardtext) ;
                    setTextColor(GREEN);
                    setCursor(40, 210) ;
                    writeString(level1text) ;
                } else if (state == 5) {
                    setCursor(40, 210) ;
                    writeString(level1text) ;
                    setTextColor(GREEN);
                    setCursor(40, 230) ;
                    writeString(level2text) ;
                } else if (state == 6) {
                    setCursor(40, 230) ;
                    writeString(level2text) ;
                    setTextColor(GREEN);
                    setCursor(40, 250) ;
                    writeString(level3text) ;
                } else if (state == 7) {
                    setCursor(40, 250) ;
                    writeString(level3text) ;
                    setTextColor(GREEN);
                    setCursor(40, 270) ;
                    writeString(level4text) ;
                }
            }
        } else if (button1_pressed == 0) {
            button1_previous = 0;
        }

        if (button2_pressed == 1 && button2_previous == 0) {
            button2_previous = 1;
            if (win) {
                win = 0;
                win_previous = 0;
                restart = 1;
                if (win_state == 0) {
                    level = (level + 1) % 4;
                }
                fillRect(plane_x0, plane_y0, plane_width + quad_size, plane_height + quad_size, BLACK) ;

                fillRect(plane_x0, 60, 120, 20, BLACK) ;
                setTextColor(MAGENTA);
                setCursor(plane_x0, 60) ;

                if (level == 0) {
                    writeString(level1text) ;
                } else if (level == 1) {
                    writeString(level2text) ;
                    drawMaze();
                } else if (level == 2) {
                    writeString(level3text) ;
                    drawMaze();
                    fillRect(trap_1_x0, trap_1_y0, endpoint_size, endpoint_size, RED) ;
                    fillRect(trap_2_x0, trap_2_y0, endpoint_size, endpoint_size, RED) ;
                } else if (level == 3) {
                    writeString(level4text) ;
                    drawMaze();
                    fillRect(trap_1_x0, trap_1_y0, endpoint_size, endpoint_size, RED) ;
                    fillRect(trap_2_x0, trap_2_y0, endpoint_size, endpoint_size, RED) ;
                }
            } else if (lose) {
                lose = 0;
                lose_previous = 0;
                restart = 1;
                fillRect(plane_x0, plane_y0, plane_width + quad_size, plane_height + quad_size, BLACK) ;
                if (level == 1) {
                    drawMaze();
                } else if (level > 1) {
                    drawMaze();
                    fillRect(trap_1_x0, trap_1_y0, endpoint_size, endpoint_size, RED) ;
                    fillRect(trap_2_x0, trap_2_y0, endpoint_size, endpoint_size, RED) ;
                }
            } else {
                if (state == 0) {
                    restart = 1;
                    fillRect(fix2int15(ph), fix2int15(pv), quad_size, quad_size, BLACK) ;
                } else if (state == 1) {
                    g = g1;
                    fillRect(plane_x0+130, 60, 100, 20, BLACK) ;
                    setTextColor(GREEN);
                    setCursor(plane_x0+130, 60) ;
                    writeString(easytext) ;
                } else if (state == 2) {
                    g = g2;
                    fillRect(plane_x0+130, 60, 100, 20, BLACK) ;
                    setTextColor(CYAN);
                    setCursor(plane_x0+130, 60) ;
                    writeString(midtext) ;
                } else if (state == 3) {
                    g = g3;
                    fillRect(plane_x0+130, 60, 100, 20, BLACK) ;
                    setTextColor(BLUE);
                    setCursor(plane_x0+130, 60) ;
                    writeString(hardtext) ;
                } else if (state == 4) {
                    if (level != 0) {
                        level = 0;
                        fillRect(plane_x0, 60, 120, 20, BLACK) ;
                        setTextColor(MAGENTA);
                        setCursor(plane_x0, 60) ;
                        writeString(level1text) ;
                        fillRect(plane_x0, plane_y0, plane_width + quad_size, plane_height + quad_size, BLACK) ;
                        restart = 1;
                    }
                } else if (state == 5) {
                    if (level != 1) {
                        level = 1;
                        fillRect(plane_x0, 60, 120, 20, BLACK) ;
                        setTextColor(MAGENTA);
                        setCursor(plane_x0, 60) ;
                        writeString(level2text) ;
                        fillRect(plane_x0, plane_y0, plane_width + quad_size, plane_height + quad_size, BLACK) ;
                        drawMaze();
                        restart = 1;
                    }
                } else if (state == 6) {
                    if (level != 2) {
                        level = 2;
                        fillRect(plane_x0, 60, 120, 20, BLACK) ;
                        setTextColor(MAGENTA);
                        setCursor(plane_x0, 60) ;
                        writeString(level3text) ;
                        fillRect(plane_x0, plane_y0, plane_width + quad_size, plane_height + quad_size, BLACK) ;
                        drawMaze();
                        fillRect(trap_1_x0, trap_1_y0, endpoint_size, endpoint_size, RED) ;
                        fillRect(trap_2_x0, trap_2_y0, endpoint_size, endpoint_size, RED) ;
                        restart = 1;
                    }
                } else if (state == 7) {
                    if (level != 3) {
                        level = 3;
                        fillRect(plane_x0, 60, 120, 20, BLACK) ;
                        setTextColor(MAGENTA);
                        setCursor(plane_x0, 60) ;
                        writeString(level4text) ;

                        fillRect(plane_x0, plane_y0, plane_width + quad_size, plane_height + quad_size, BLACK) ;
                        drawMaze();
                        fillRect(trap_1_x0, trap_1_y0, endpoint_size, endpoint_size, RED) ;
                        fillRect(trap_2_x0, trap_2_y0, endpoint_size, endpoint_size, RED) ;
                        restart = 1;
                    }
                }
            }
        } else if (button2_pressed == 0) {
            button2_previous = 0;
        }

        if (win) {
            if (win_previous == 0) {
                win_previous = 1;
                fillRect(plane_x0, plane_y0, plane_width + quad_size, plane_height + quad_size, BLACK) ;
                setTextSize(5) ;
                setTextColor(MAGENTA);
                setCursor(250, 200) ;
                writeString(youwintext) ;
                setTextSize(2) ;
                setTextColor(GREEN);
                setCursor(320, 260) ;
                writeString(nextleveltext) ;
                setTextColor(WHITE);
                setCursor(320, 290) ;
                writeString(restarttext) ;
            }
        } else if (lose) {
            if (lose_previous == 0) {
                lose_previous = 1;
                fillRect(plane_x0, plane_y0, plane_width + quad_size, plane_height + quad_size, BLACK) ;
                setTextSize(5) ;
                setTextColor(YELLOW);
                setCursor(200, 200) ; // ??
                writeString(youlosetext) ;
                setTextSize(2) ;
                setTextColor(GREEN);
                setCursor(320, 280) ; // ??
                writeString(restarttext) ;
            }
        } else {
            fillRect(endpoint_1_x0, endpoint_1_y0, endpoint_size, endpoint_size, GREEN) ;
            fillRect(plane_x0+320, 60, 120, 20, BLACK) ;
            if (level == 2) {
                fillRect(trap_1_x0, trap_1_y0, endpoint_size, endpoint_size, RED) ;
                fillRect(trap_2_x0, trap_2_y0, endpoint_size, endpoint_size, RED) ;
            } else if (level == 3) {
                fillRect(trap_1_x0, trap_1_y0, endpoint_size, endpoint_size, RED) ;
                fillRect(trap_2_x0, trap_2_y0, endpoint_size, endpoint_size, RED) ;

                sprintf(timetext, "%.3f", level4_time) ;
                setTextColor(YELLOW);
                setCursor(plane_x0+320, 60) ;
                writeString(timetext) ;
            }
        }

        // Yield for 25 ms
        PT_YIELD_usec(25000) ;
    }
    // Indicate end of thread
    PT_END(pt);
}

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){ 
  //
  //  === add threads  ====================
  // for core 1
    //
    // put slow threads on core 1
    //pt_add_thread(protothread_vga) ;
    //
    pt_schedule_start ;
  // NEVER exits
  // ======================================
}

// server strunct used below to set up AP
typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    bool complete;
    ip_addr_t gw;
    async_context_t *context;
} TCP_SERVER_T;

// ====================================================
int main() {
  // you may want to change these for privacy
  #define WIFI_SSID     "picow_test"
  #define WIFI_PASSWORD "password"
  // you may want to change these to avoid intrference
  #define STATION_ADDR "192.168.4.10"
  #define AP_ADDR      "192.168.4.1"

  // =======================
  // init the serial
    stdio_init_all();

    // connecting gpio2 to Vdd sets 'AP' Access Point
    #define mode_sel 2
    gpio_init(mode_sel) ;	
    gpio_set_dir(mode_sel, GPIO_IN) ;
    // turn pulldown on
    gpio_set_pulls (mode_sel, false, true) ;
   
   // =======================
   // choose station vs access point
   // 
   int ap ;
   // jumper gpio 2 high for AP
   // start AP unit first!
   ap = gpio_get(mode_sel) ;
   //
   if(ap){
      TCP_SERVER_T *state = calloc(1, sizeof(TCP_SERVER_T));
    if (!state) {
        printf("failed to allocate state\n");
        return 1;
    }

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }
    // access point SSID and PASSWORD
    // WPA2 authorization
    // now turn on AP mode
    cyw43_arch_enable_ap_mode(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);

    // 'state' is a pointer to type TCP_SERVER_T 
    // set up the access point IP address and mask
    // 
    ip4_addr_t mask;
    ip4_addr_t ap_addr;
    // arbirary access point IP address is AP_ADDR
    ipaddr_aton(AP_ADDR, &ap_addr);
    // set the server struct address field
    state->gw = ap_addr ;
    //IP4_ADDR(ip_2_ip4(&state->gw), 192, 168, 4, 1);
    // set the network mask
    IP4_ADDR(ip_2_ip4(&mask), 255, 255, 255, 0);

    //station address for data send (as set below)
    sprintf(udp_target_pico,"%s", STATION_ADDR);

    // Start the dhcp server
    // Even though in this program DHCP is not required, LWIP
    // seems to need it!
    // and set picoW IP address from 'state' structure
    // set 'mask' as defined above
    dhcp_server_t dhcp_server;
    dhcp_server_init(&dhcp_server, &state->gw, &mask);
    }
  
  else { // Station mode
    sleep_ms(1000) ;
    // =======================
  // init the staTION network
    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

    // hook up to local WIFI
    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        // and use known ap target from above to send string
        sprintf(udp_target_pico,"%s", AP_ADDR);

        // set local addr by overridding DHCP
        //ip_addr_t ip;
        // must be STATION_ADDR
        ip_addr_t sta_addr;
        ipaddr_aton(STATION_ADDR, &sta_addr);
        netif_set_ipaddr(netif_default, &sta_addr);
        //IP4_ADDR(&ip, 192,168,4,10);
        //netif_set_ipaddr(netif_default, &ip);
    }

  }

    // Initialize VGA
    initVGA() ;

  
    //====================================
    // set up UDP recenve ISR handler
    udpecho_raw_init();

  // =====================================
  // init the thread control semaphores
  // for the send/receive
  // recv semaphone is set by an ISR
  PT_SEM_INIT(&new_udp_send_s, 0) ;
  PT_SEM_INIT(&new_udp_recv_s, 0) ;
  // allow one thread at a time to use UART
  //PT_SEM_INIT(&use_uart_s, 1);

  // =====================================
  // core 1
  // start core 1 threads
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // === config threads ========================
  // for core 0

  pt_add_thread(protothread_udp_recv);
  pt_add_thread(protothread_vga) ;
  //pt_add_thread(protothread_udp_send);
  //
  // === initalize the scheduler ===============
  pt_schedule_start ;

    cyw43_arch_deinit();
    return 0;
}