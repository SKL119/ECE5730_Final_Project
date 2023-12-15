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

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];
fix15 accel_x, accel_y, accel_z;
float angle_v_rad, angle_h_rad;
fix15 cos_angle_v, cos_angle_h;

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

fix15 data[5];
int button1_pressed = 0;
int button2_pressed = 0;

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

// =======================================
// UDP send thead
// actually sends data when signalled
// =======================================
static ip_addr_t addr;
static PT_THREAD (protothread_udp_send(struct pt *pt)) {
    PT_BEGIN(pt);
    static struct udp_pcb* pcb;
    pcb = udp_new();
    pcb->remote_port = UDP_PORT ;
    pcb->local_port = UDP_PORT ;

    static int counter = 0;
    
    while (true) {
        
        // stall until there is actually something to send
        PT_SEM_WAIT(pt, &new_udp_send_s) ;

        // 'main' sets up target addr for each pico
        ipaddr_aton(udp_target_pico, &addr);
                
        // arbitrary length for this example
        static int udp_send_length = 128 ;
        
       // actual data-send
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, udp_send_length+1, PBUF_RAM);
        char *req = (char *)p->payload;
        memset(req, 0, udp_send_length+1);//
        memcpy(req, send_data, udp_send_length) ;
        // memset(req, 0, udp_send_length+1);//
        // memcpy(req, send_data, 5*__SIZEOF_INT__) ;
        // the SEND
        err_t er = udp_sendto(pcb, p, &addr, UDP_PORT); //port
        // free the data structure
        pbuf_free(p);
        if (er != ERR_OK) {
            printf("Failed to send UDP packet! error=%d", er);
        } else {
           // printf("Sent packet %d\n", counter);
            counter++;
        }
    }
    PT_END(pt);
}


// This timer ISR is called on core 0
bool repeating_timer_callback_core_0(struct repeating_timer *t) {

    // ============================ Button ============================
    if (gpio_get(GPIO_BUTTON1) == 0) {
        button1_pressed = 1;
    } else {
        button1_pressed = 0;
    }

    if (gpio_get(GPIO_BUTTON2) == 0) {
        button2_pressed = 1;
    } else {
        button2_pressed = 0;
    }

    data[3] = int2fix15(button1_pressed);
    data[4] = int2fix15(button2_pressed);

    // ============================ IMU ============================
    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // ============================ Plane Tilt Angle ============================

    // Accelerometer angle (degrees - 15.16 fixed point) 
    // Only ONE of the two lines below will be used, depending whether or not a small angle approximation is appropriate
    // SMALL ANGLE APPROXIMATION
    // accel_angle = multfix15(divfix(acceleration[0], acceleration[1]), oneeightyoverpi) ;
    // NO SMALL ANGLE APPROXIMATION

    accel_x = acceleration[0];
    accel_y = acceleration[1];
    accel_z = acceleration[2];

    // float
    angle_v_rad = atan2(-accel_x, accel_z) + M_PI/2;
    angle_h_rad = atan2(-accel_y, accel_z) + M_PI/2;

    cos_angle_v = float2fix15(cos(angle_v_rad));
    cos_angle_h = float2fix15(cos(angle_h_rad));

    data[1] = cos_angle_v;
    data[2] = cos_angle_h;

    // ============================ Prepare Send Data ============================
    memset(send_data, 0, UDP_MSG_LEN_MAX) ;
    memcpy(send_data, data, 5*__SIZEOF_INT__) ;
    // set the array data flag
    send_data[0] = 0xee ;

    PT_SEM_SIGNAL(pt, &new_udp_send_s) ;

    return true;
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
    // pt_add_thread(protothread_toggle_cyw43) ;
    // pt_add_thread(protothread_serial) ;
    pt_add_thread(protothread_udp_send);
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
    } else { // Station mode
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

    // Map GPIO_BUTTON1 to GPIO port, make it low
    gpio_init(GPIO_BUTTON1) ;
    gpio_set_dir(GPIO_BUTTON1, GPIO_IN) ;
    gpio_pull_up(GPIO_BUTTON1);

    gpio_init(GPIO_BUTTON2) ;
    gpio_set_dir(GPIO_BUTTON2, GPIO_IN) ;
    gpio_pull_up(GPIO_BUTTON2);


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    // gpio_pull_up(SDA_PIN) ;
    // gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);
  
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

  // Create a repeating timer that calls 
  // repeating_timer_callback (defaults core 0)
  struct repeating_timer timer_core_0;

  // Negative delay so means we will call repeating_timer_callback, and call it
  // again 25us (40kHz) later regardless of how long the callback took to execute
  // 2500 us = 2.5 ms => 400 Hz
  // 25000 us = 25 ms => 40 Hz
  add_repeating_timer_us(-25000, repeating_timer_callback_core_0, NULL, &timer_core_0);

  //pt_add_thread(protothread_udp_recv);
  // pt_add_thread(protothread_udp_send);
  //
  // === initalize the scheduler ===============
  pt_schedule_start ;

    cyw43_arch_deinit();
    return 0;
}