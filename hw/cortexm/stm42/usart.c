/*
 * STM32 USART
 *
 * Copyright (c) 2015 Dimitri L. <dimdimdimdim at gmx dot fr>
 * Includes substantial work from:
 * 	Copyright (c) 2015 Liviu Ionescu (stm32-gpio.c)
 * 	Copyright (c) 2014 Alistair Francis (stm32f2xx_usart.c)
 * 	Copyright (c) 2010 Andre Beckus (stm32-gpio.c)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <hw/cortexm/stm42/usart.h>
#include <hw/cortexm/stm42/mcu.h>
#include <hw/cortexm/svd.h>

#include "sysemu/sysemu.h"

#define USART_SR_TXE        (1 << 7)
#define USART_SR_TC         (1 << 6)
#define USART_SR_RXNE       (1 << 5)

#define USART_CR1_UE        (1 << 13)
#define USART_CR1_TXEIE     (1 << 7)
#define USART_CR1_TCIE      (1 << 6)
#define USART_CR1_RXNEIE    (1 << 5)
#define USART_CR1_TE        (1 << 3)
#define USART_CR1_RE        (1 << 2)

// ----- Generated code -------------------------------------------------------
//-------------------------------------------------------------------

// STM32F407VG, STM32F407ZG, STM32F405RG
// DO NOT EDIT! Automatically generated!
static void stm32f40x_usart_create_objects(Object *obj, JSON_Object *svd,
        const char *name)
{
    STM32USARTState *state = STM32_USART_STATE(obj);

    JSON_Object *periph = svd_get_peripheral_by_name(svd, name);
    svd_add_peripheral_properties_and_children(obj, periph, svd);

    // Registers.
    state->u.f4.reg.sr = cm_object_get_child_by_name(obj, "SR");
    state->u.f4.reg.dr = cm_object_get_child_by_name(obj, "DR");
    state->u.f4.reg.brr = cm_object_get_child_by_name(obj, "BRR");
    state->u.f4.reg.cr1 = cm_object_get_child_by_name(obj, "CR1");
    state->u.f4.reg.cr2 = cm_object_get_child_by_name(obj, "CR2");
    state->u.f4.reg.cr3 = cm_object_get_child_by_name(obj, "CR3");
    state->u.f4.reg.gtpr = cm_object_get_child_by_name(obj, "GTPR");

    // SR bitfields.
    state->u.f4.fld.sr.pe = cm_object_get_child_by_name(state->u.f4.reg.sr,
            "PE");
    state->u.f4.fld.sr.fe = cm_object_get_child_by_name(state->u.f4.reg.sr,
            "FE");
    state->u.f4.fld.sr.nf = cm_object_get_child_by_name(state->u.f4.reg.sr,
            "NF");
    state->u.f4.fld.sr.ore = cm_object_get_child_by_name(state->u.f4.reg.sr,
            "ORE");
    state->u.f4.fld.sr.idle = cm_object_get_child_by_name(state->u.f4.reg.sr,
            "IDLE");
    state->u.f4.fld.sr.rxne = cm_object_get_child_by_name(state->u.f4.reg.sr,
            "RXNE");
    state->u.f4.fld.sr.tc = cm_object_get_child_by_name(state->u.f4.reg.sr,
            "TC");
    state->u.f4.fld.sr.txe = cm_object_get_child_by_name(state->u.f4.reg.sr,
            "TXE");
    state->u.f4.fld.sr.lbd = cm_object_get_child_by_name(state->u.f4.reg.sr,
            "LBD");
    state->u.f4.fld.sr.cts = cm_object_get_child_by_name(state->u.f4.reg.sr,
            "CTS");

    // DR bitfields.
    state->u.f4.fld.dr.dr = cm_object_get_child_by_name(state->u.f4.reg.dr,
            "DR");

    // BRR bitfields.
    state->u.f4.fld.brr.div_fraction = cm_object_get_child_by_name(
            state->u.f4.reg.brr, "DIV_Fraction");
    state->u.f4.fld.brr.div_mantissa = cm_object_get_child_by_name(
            state->u.f4.reg.brr, "DIV_Mantissa");

    // CR1 bitfields.
    state->u.f4.fld.cr1.sbk = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "SBK");
    state->u.f4.fld.cr1.rwu = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "RWU");
    state->u.f4.fld.cr1.re = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "RE");
    state->u.f4.fld.cr1.te = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "TE");
    state->u.f4.fld.cr1.idleie = cm_object_get_child_by_name(
            state->u.f4.reg.cr1, "IDLEIE");
    state->u.f4.fld.cr1.rxneie = cm_object_get_child_by_name(
            state->u.f4.reg.cr1, "RXNEIE");
    state->u.f4.fld.cr1.tcie = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "TCIE");
    state->u.f4.fld.cr1.txeie = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "TXEIE");
    state->u.f4.fld.cr1.peie = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "PEIE");
    state->u.f4.fld.cr1.ps = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "PS");
    state->u.f4.fld.cr1.pce = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "PCE");
    state->u.f4.fld.cr1.wake = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "WAKE");
    state->u.f4.fld.cr1.m = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "M");
    state->u.f4.fld.cr1.ue = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "UE");
    state->u.f4.fld.cr1.over8 = cm_object_get_child_by_name(state->u.f4.reg.cr1,
            "OVER8");

    // CR2 bitfields.
    state->u.f4.fld.cr2.add = cm_object_get_child_by_name(state->u.f4.reg.cr2,
            "ADD");
    state->u.f4.fld.cr2.lbdl = cm_object_get_child_by_name(state->u.f4.reg.cr2,
            "LBDL");
    state->u.f4.fld.cr2.lbdie = cm_object_get_child_by_name(state->u.f4.reg.cr2,
            "LBDIE");
    state->u.f4.fld.cr2.lbcl = cm_object_get_child_by_name(state->u.f4.reg.cr2,
            "LBCL");
    state->u.f4.fld.cr2.cpha = cm_object_get_child_by_name(state->u.f4.reg.cr2,
            "CPHA");
    state->u.f4.fld.cr2.cpol = cm_object_get_child_by_name(state->u.f4.reg.cr2,
            "CPOL");
    state->u.f4.fld.cr2.clken = cm_object_get_child_by_name(state->u.f4.reg.cr2,
            "CLKEN");
    state->u.f4.fld.cr2.stop = cm_object_get_child_by_name(state->u.f4.reg.cr2,
            "STOP");
    state->u.f4.fld.cr2.linen = cm_object_get_child_by_name(state->u.f4.reg.cr2,
            "LINEN");

    // CR3 bitfields.
    state->u.f4.fld.cr3.eie = cm_object_get_child_by_name(state->u.f4.reg.cr3,
            "EIE");
    state->u.f4.fld.cr3.iren = cm_object_get_child_by_name(state->u.f4.reg.cr3,
            "IREN");
    state->u.f4.fld.cr3.irlp = cm_object_get_child_by_name(state->u.f4.reg.cr3,
            "IRLP");
    state->u.f4.fld.cr3.hdsel = cm_object_get_child_by_name(state->u.f4.reg.cr3,
            "HDSEL");
    state->u.f4.fld.cr3.nack = cm_object_get_child_by_name(state->u.f4.reg.cr3,
            "NACK");
    state->u.f4.fld.cr3.scen = cm_object_get_child_by_name(state->u.f4.reg.cr3,
            "SCEN");
    state->u.f4.fld.cr3.dmar = cm_object_get_child_by_name(state->u.f4.reg.cr3,
            "DMAR");
    state->u.f4.fld.cr3.dmat = cm_object_get_child_by_name(state->u.f4.reg.cr3,
            "DMAT");
    state->u.f4.fld.cr3.rtse = cm_object_get_child_by_name(state->u.f4.reg.cr3,
            "RTSE");
    state->u.f4.fld.cr3.ctse = cm_object_get_child_by_name(state->u.f4.reg.cr3,
            "CTSE");
    state->u.f4.fld.cr3.ctsie = cm_object_get_child_by_name(state->u.f4.reg.cr3,
            "CTSIE");
    state->u.f4.fld.cr3.onebit = cm_object_get_child_by_name(
            state->u.f4.reg.cr3, "ONEBIT");

    // GTPR bitfields.
    state->u.f4.fld.gtpr.psc = cm_object_get_child_by_name(state->u.f4.reg.gtpr,
            "PSC");
    state->u.f4.fld.gtpr.gt = cm_object_get_child_by_name(state->u.f4.reg.gtpr,
            "GT");
}

// ----------------------------------------------------------------------------

// ----- Public ---------------------------------------------------------------

// Create GPIO%c and return it.
Object* stm32_usart_create(Object *parent, stm32_usart_index_t index)
{
    if ((int) index >= STM32_PORT_USART_UNDEFINED) {
        hw_error("Cannot assign USART %d: QEMU supports only %d ports\n",
                1 + index - STM32_PORT_USART1, STM32_PORT_USART_UNDEFINED);
    }

    char child_name[10];
    snprintf(child_name, sizeof(child_name) - 1, "USART%d",
            1 + index - STM32_PORT_USART1);
    // Passing a local string is ok.
    Object *usart = cm_object_new(parent, child_name,
    TYPE_STM32_USART);

    object_property_set_int(usart, index, "port-index", NULL);

    cm_object_realize(usart);

    return usart;
}

// ----------------------------------------------------

static bool stm32_usart_is_enabled(Object *obj)
{
    STM32USARTState *state = STM32_USART_STATE(obj);

    if (register_bitfield_is_non_zero(state->enabling_bit)) {
        return true; // Positive logic, bit == 1 means enabled.
    }

    // Not enabled
    return false;
}

// ----------------------------------------------------------------------------

static int stm32f4_usart_get_irq_vector(STM32USARTState *state)
{
    // TODO: use capabilities to select interrupt numbers
    // for different variants.

    switch (state->port_index) {
    case STM32_PORT_USART1:
        return STM32F4_01_57_XX_USART1_IRQn;
    case STM32_PORT_USART2:
        return STM32F4_01_57_XX_USART2_IRQn;
    case STM32_PORT_USART3:
        return STM32F4_01_57_XX_USART3_IRQn;
    case STM32_PORT_USART4:
        return STM32F4_01_57_XX_UART4_IRQn;
    case STM32_PORT_USART5:
        return STM32F4_01_57_XX_UART5_IRQn;
    case STM32_PORT_USART6:
        return STM32F4_01_57_XX_USART6_IRQn;
    default:
        return 1023; // Whatever...
    }
}

/* HELPER FUNCTIONS */

/* Update the baud rate based on the USART's peripheral clock frequency. */
static void stm32_uart_baud_update(void *obj)
{
  STM32USARTState *state = STM32_USART_STATE((Object * )obj);

      int32_t brr = peripheral_register_get_raw_value(state->reg.brr);
      brr = brr;
/*    uint32_t clk_freq = stm32_rcc_get_periph_freq(s->stm32_rcc, s->periph);

    uint64_t ns_per_bit;

    if((s->USART_BRR == 0) || (clk_freq == 0)) {
        s->bits_per_sec = 0;
    } else {
        s->bits_per_sec = clk_freq / s->USART_BRR;
        ns_per_bit = 1000000000LL / s->bits_per_sec;

        // We assume 10 bits per character.  This may not be exactly
        // accurate depending on settings, but it should be good enough.
        s->ns_per_char = ns_per_bit * 10;
    }

#ifdef DEBUG_STM32_UART
    DPRINTF("%s clock is set to %lu Hz.\n",
                stm32_periph_name(s->periph),
                (unsigned long)clk_freq);
    DPRINTF("%s BRR set to %lu.\n",
                stm32_periph_name(s->periph),
                (unsigned long)s->USART_BRR);
    DPRINTF("%s Baud is set to %lu bits per sec.\n",
                stm32_periph_name(s->periph),
                (unsigned long)s->bits_per_sec);
#endif
*/
}

/* Handle a change in the peripheral clock. */
static void stm32_uart_clk_irq_handler(void *opaque, int n, int level)
{
    //STM32USARTState *state = STM32_USART_STATE((Object * )opaque);

    assert(n == 0);

    /* Only update the BAUD rate if the IRQ is being set. */
    if(level) {
        stm32_uart_baud_update(opaque);
    }
}

/* Routine which updates the USART's IRQ.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void stm32_uart_update_irq(void *obj) {
    /* Note that we are not checking the ORE flag, but we should be. */
    STM32USARTState *state = STM32_USART_STATE((Object * )obj);
    int32_t cr1 = peripheral_register_get_raw_value(state->reg.cr1);
    int32_t sr = peripheral_register_get_raw_value(state->reg.sr);
    int32_t cr1_tcie = (cr1 & USART_CR1_TCIE) ? 1 : 0;
    int32_t cr1_txeie = (cr1 & USART_CR1_TXEIE) ? 1 : 0;
    int32_t cr1_rxneie = (cr1 & USART_CR1_RXNEIE) ? 1 : 0;
    int32_t sr_tc = (sr & USART_SR_TC) ? 1 : 0;
    int32_t sr_txe = (sr & USART_SR_TXE) ? 1 : 0;
    //int32_t sr_ore = (sr & USART_SR_ORE) ? 1 : 0;
    int32_t sr_rxne = (sr & USART_SR_RXNE) ? 1 : 0;
    int new_irq_level =
       (cr1_tcie & sr_tc) |
       (cr1_txeie & sr_txe) |
       (cr1_rxneie &
               //(sr_ore | sr_rxne));
               sr_rxne);
    /* Only trigger an interrupt if the IRQ level changes.  We probably could
     * set the level regardless, but we will just check for good measure.
     */
    if(new_irq_level ^ state->curr_irq_level) {
        qemu_set_irq(state->irq, new_irq_level);
        state->curr_irq_level = new_irq_level;
        //cortexm_nvic_set_pending_interrupt(state->nvic,
        //        stm32f4_usart_get_irq_vector(state));
    }
}

static void stm32_uart_start_tx(void *obj, uint32_t value);

/* Routine to be called when a transmit is complete. */
static void stm32_uart_tx_complete(void *obj)
{
    STM32USARTState *state = STM32_USART_STATE((Object * )obj);

    int32_t sr = peripheral_register_get_raw_value(state->reg.sr);
    uint32_t dr = peripheral_register_get_raw_value(state->reg.dr);
    int32_t sr_txe = (sr & USART_SR_TXE) ? 1 : 0;
    //int32_t sr_tc = (sr & USART_SR_TC) ? 1 : 0;
    if(sr_txe) {
        /* If the buffer is empty, there is nothing waiting to be transmitted.
         * Mark the transmit complete. */
        peripheral_register_or_raw_value(state->reg.sr, USART_SR_TC);
        stm32_uart_update_irq(obj);
    } else {
        /* Otherwise, mark the transmit buffer as empty and
         * start transmitting the value stored there.
         */
        peripheral_register_or_raw_value(state->reg.sr, USART_SR_TXE);
        stm32_uart_update_irq(obj);
        stm32_uart_start_tx(obj, dr);
    }
}

/* Start transmitting a character. */
static void stm32_uart_start_tx(void *obj, uint32_t value)
{
    uint8_t ch = value; //This will truncate the ninth bit

    /* Reset the Transmission Complete flag to indicate a transmit is in
     * progress.
     */
    STM32USARTState *state = STM32_USART_STATE((Object * )obj);
    //int32_t sr = peripheral_register_get_raw_value(state->reg.sr);
    int32_t n_sr_tc = ~USART_SR_TC;
    //s->USART_SR_TC = 0;
    peripheral_register_and_raw_value(state->reg.sr, n_sr_tc);

    /* Write the character out. */
    if (state->chr) {
        qemu_chr_fe_write(state->chr, &ch, 1);
    }
    stm32_uart_tx_complete(obj);
}


static int stm32f4_usart_can_receive(void *obj)
{
    STM32USARTState *state = STM32_USART_STATE((Object * )obj);

    int32_t sr = peripheral_register_get_raw_value(state->reg.sr);
    int32_t cr1 = peripheral_register_get_raw_value(state->reg.cr1);
    int32_t cr1_ue = (cr1 & USART_CR1_UE) ? 1 : 0;
    int32_t cr1_re = (cr1 & USART_CR1_RE) ? 1 : 0;
    int32_t sr_rxne = (sr & USART_SR_RXNE) ? 1 : 0;
    if(cr1_ue & cr1_re & stm32_usart_is_enabled(obj)) {
      if(sr_rxne) {
        return 0;
      } else {
        return 1;
      }
    } else {
      return 1;
    }
}

static void stm32f4_usart_receive(void *obj, const uint8_t *buf, int size)
{
    STM32USARTState *state = STM32_USART_STATE((Object * )obj);

    int32_t cr1 = peripheral_register_get_raw_value(state->reg.cr1);
    int32_t cr1_rxneie = (cr1 & USART_CR1_RXNEIE) ? 1 : 0;
    //int32_t sr = peripheral_register_get_raw_value(state->reg.sr);
    //int32_t sr_rxne = (sr & USART_SR_RXNE) ? 1 : 0;

    peripheral_register_set_raw_value(state->reg.dr, *buf);
    peripheral_register_or_raw_value(state->reg.sr, USART_SR_RXNE);

    if (cr1_rxneie) {
        stm32_uart_update_irq(obj);
    }
}

static void stm32f4_usart_event(void *opaque, int event)
{
    /* Do nothing */
}

/* REGISTER IMPLEMENTATION */

static uint32_t stm32_uart_USART_SR_read(void *obj)
{
  STM32USARTState *state = STM32_USART_STATE((Object * )obj);

  uint32_t sr = peripheral_register_get_raw_value(state->reg.sr);

  return sr;
    /* If the Overflow flag is set, reading the SR register is the first step
     * to resetting the flag.
     */
    //if(s->USART_SR_ORE) {
    //    s->sr_read_since_ore_set = true;
    //}

    //return (s->USART_SR_TXE << USART_SR_TXE_BIT) |
    //       (s->USART_SR_TC << USART_SR_TC_BIT) |
    //       (s->USART_SR_RXNE << USART_SR_RXNE_BIT) |
    //       (s->USART_SR_ORE << USART_SR_ORE_BIT);
}





static void stm32_uart_USART_SR_write(void *obj, uint32_t new_value)
{
    STM32USARTState *state = STM32_USART_STATE((Object * )obj);

    uint32_t new_TC = (new_value & USART_SR_TC) ? 1 : 0;
    uint32_t new_RXNE = (new_value & USART_SR_RXNE) ? 1 : 0;
    int32_t n_sr_tc =  ~USART_SR_TC;
    int32_t n_sr_rxne =  ~USART_SR_RXNE;

    assert(new_TC==0);
    assert(new_RXNE==0);

    //s->USART_SR_TC = new_TC;
    peripheral_register_and_raw_value(state->reg.sr, n_sr_tc);
    //s->USART_SR_RXNE = new_RXNE;
    peripheral_register_and_raw_value(state->reg.sr, n_sr_rxne);

    stm32_uart_update_irq(obj);
}


static void stm32_uart_USART_DR_read(void *obj, uint32_t *read_value)
{
    STM32USARTState *state = STM32_USART_STATE((Object * )obj);

    int32_t cr1 = peripheral_register_get_raw_value(state->reg.cr1);
    int32_t cr1_ue = (cr1 & USART_CR1_UE) ? 1 : 0;
    int32_t cr1_re = (cr1 & USART_CR1_RE) ? 1 : 0;
    int32_t sr = peripheral_register_get_raw_value(state->reg.sr);
    int32_t sr_rxne = (sr & USART_SR_RXNE) ? 1 : 0;
    int32_t n_sr_rxne =  ~USART_SR_RXNE;
    uint32_t dr = peripheral_register_get_raw_value(state->reg.dr);

    if(sr_rxne & cr1_ue & cr1_re) {
        /* If the receive buffer is not empty, return the value. and mark the
         * buffer as empty.
         */
        (*read_value) = dr;
        peripheral_register_and_raw_value(state->reg.sr, n_sr_rxne);
        stm32_uart_update_irq(obj);
    } else {
        return;
    }

}


static void stm32_uart_USART_DR_write(void *obj, uint32_t new_value)
{
    STM32USARTState *state = STM32_USART_STATE((Object * )obj);

    int32_t cr1 = peripheral_register_get_raw_value(state->reg.cr1);
    int32_t cr1_ue = (cr1 & USART_CR1_UE) ? 1 : 0;
    int32_t cr1_te = (cr1 & USART_CR1_TE) ? 1 : 0;
    int32_t sr = peripheral_register_get_raw_value(state->reg.sr);
    int32_t sr_tc = (sr & USART_SR_TC) ? 1 : 0;
    int32_t sr_txe = (sr & USART_SR_TXE) ? 1 : 0;
    int32_t n_sr_txe =  ~USART_SR_TXE;

    uint32_t write_value = new_value & 0x000001ff;

    //stm32_uart_check_tx_pin(s);
    if(cr1_ue & cr1_te) {
      if(sr_tc) {
          /* If the Transmission Complete bit is set, it means the USART is not
           * currently transmitting.  This means, a transmission can immediately
           * start.
           */
          stm32_uart_start_tx(obj, write_value);
      } else {
          /* Otherwise check to see if the buffer is empty.
           * If it is, then store the new character there and mark it as not empty.
           * If it is not empty, trigger a hardware error.  Software should check
           * to make sure it is empty before writing to the Data Register.
           */
          if(sr_txe) {
              //s->USART_TDR = write_value;
              peripheral_register_set_raw_value(state->reg.dr, write_value);
              //s->USART_SR_TXE = 0;
              peripheral_register_and_raw_value(state->reg.sr, n_sr_txe);
          } else {
            return;
          }
      }

    } else {
      return;
    }

    stm32_uart_update_irq(obj);
}

/* Update the Baud Rate Register. */
static void stm32_uart_USART_BRR_write(void *obj, uint32_t new_value,
                                        bool init)
{
    //s->USART_BRR = new_value & 0x0000ffff;

    stm32_uart_baud_update(obj);
}

static void stm32_uart_USART_CR1_write(void *obj, uint32_t new_value,
                                        bool init)
{
    STM32USARTState *state = STM32_USART_STATE((Object * )obj);
    //uint32_t new_ue = (new_value & USART_CR1_UE) ? 1 : 0;
    //uint32_t new_txeie = (new_value & USART_CR1_TXEIE) ? 1 : 0;
    //uint32_t new_tcie = (new_value & USART_CR1_TCIE) ? 1 : 0;
    //uint32_t new_rxneie = (new_value & USART_CR1_RXNEIE) ? 1 : 0;
    //uint32_t new_te = (new_value & USART_CR1_TE) ? 1 : 0;
    //uint32_t new_re = (new_value & USART_CR1_RE) ? 1 : 0;
    //s->USART_CR1 = new_value & 0x00003fff;
    peripheral_register_set_raw_value(state->reg.cr1, new_value & 0x00003fff);
    stm32_uart_update_irq(obj);
}

static void stm32_uart_USART_CR2_write(void *obj, uint32_t new_value,
                                        bool init)
{
    STM32USARTState *state = STM32_USART_STATE((Object * )obj);
    //s->USART_CR2 = new_value & 0x00007f7f;
    peripheral_register_set_raw_value(state->reg.cr2, new_value & 0x00007f7f);
}

static void stm32_uart_USART_CR3_write(void *obj, uint32_t new_value,
                                        bool init)
{
    STM32USARTState *state = STM32_USART_STATE((Object * )obj);
    //s->USART_CR3 = new_value & 0x000007ff;
    peripheral_register_set_raw_value(state->reg.cr3, new_value & 0x000007ff);
}




static void stm32f4_usart_dr_post_read_callback(Object *reg, Object *periph,
        uint32_t addr, uint32_t offset, unsigned size)
{
    STM32USARTState *state = STM32_USART_STATE(periph);

    peripheral_register_and_raw_value(state->reg.sr, ~USART_SR_RXNE);
    if (state->chr) {
#if 1
        qemu_chr_fe_accept_input(state->chr);
#endif
    }
}

static void stm32f4_usart_dr_post_write_callback(Object *reg, Object *periph,
        uint32_t addr, uint32_t offset, unsigned size,
        peripheral_register_t value, peripheral_register_t full_value)
{
    STM32USARTState *state = STM32_USART_STATE(periph);
    unsigned char ch;
    int32_t cr1 = peripheral_register_get_raw_value(state->reg.cr1);
    int32_t cr1_ue = (cr1 & USART_CR1_UE) ? 1 : 0;
    int32_t cr1_te = (cr1 & USART_CR1_TE) ? 1 : 0;
    int32_t cr1_txeie = (cr1 & USART_CR1_TXEIE) ? 1 : 0;
    int32_t cr1_tcie = (cr1 & USART_CR1_TCIE) ? 1 : 0;
    //int32_t sr = peripheral_register_get_raw_value(state->reg.sr);
    //int32_t sr_tc = (sr & USART_SR_TC) ? 1 : 0;
    //int32_t sr_txe = (sr & USART_SR_TXE) ? 1 : 0;
    //int32_t n_sr_txe =  ~USART_SR_TXE;

    // 'value' may be half-word, use full_word.
    if (cr1_ue & cr1_te) {
        if (state->chr) {
            ch = full_value; /* Use only the lower 8 bits */
#if 1
            qemu_chr_fe_write(state->chr, &ch, 1);
#endif
        }
        // transmission is immediately complete
        peripheral_register_or_raw_value(state->reg.sr,
        USART_SR_TC | USART_SR_TXE);
        if (cr1_txeie | cr1_tcie) {
            stm32_uart_update_irq(periph);
        }
    }
}

static void stm32f4_usart_cr1_post_write_callback(Object *reg, Object *periph,
        uint32_t addr, uint32_t offset, unsigned size,
        peripheral_register_t value, peripheral_register_t full_value)
{
    STM32USARTState *state = STM32_USART_STATE(periph);

    int32_t sr = peripheral_register_get_raw_value(state->reg.sr);

    // 'value' may be half-word, use full_word.
    if (((full_value & USART_CR1_RXNEIE) && (sr & USART_SR_RXNE))
            || ((full_value & USART_CR1_TXEIE) && (sr & USART_SR_TXE))
            || ((full_value & USART_CR1_TCIE) && (sr & USART_SR_TC))) {
        stm32_uart_update_irq(periph);
    }
}

// ----------------------------------------------------------------------------

static void stm32_usart_instance_init_callback(Object *obj)
{
    qemu_log_function_name();

    STM32USARTState *state = STM32_USART_STATE(obj);

    // FIXME use a qdev char-device prop instead of qemu_char_get_next_serial()
    // state->chr = qemu_char_get_next_serial();

    cm_object_property_add_int(obj, "port-index",
            (const int *) &state->port_index);
    state->port_index = STM32_PORT_USART_UNDEFINED;

}

static void stm32_usart_realize_callback(DeviceState *dev, Error **errp)
{
    qemu_log_function_name();

    // Call parent realize().
    if (!cm_device_parent_realize(dev, errp, TYPE_STM32_USART)) {
        return;
    }

    STM32MCUState *mcu = stm32_mcu_get();
    CortexMState *cm_state = CORTEXM_MCU_STATE(mcu);

    STM32USARTState *state = STM32_USART_STATE(dev);
    // First thing first: get capabilities from MCU, needed everywhere.
    state->capabilities = mcu->capabilities;

    Object *obj = OBJECT(dev);

    state->nvic = CORTEXM_NVIC_STATE(cm_state->nvic);

    char periph_name[10];
    snprintf(periph_name, sizeof(periph_name) - 1, "USART%d",
            1 + state->port_index - STM32_PORT_USART1);

    svd_set_peripheral_address_block(cm_state->svd_json, periph_name, obj);
    peripheral_create_memory_region(obj);

    // Must be defined before creating registers.
    cm_object_property_set_int(obj, 4, "register-size-bytes");

    // TODO: get it from MCU
    cm_object_property_set_bool(obj, true, "is-little-endian");

    const STM32Capabilities *capabilities =
    STM32_USART_STATE(state)->capabilities;
    assert(capabilities != NULL);

    char enabling_bit_name[STM32_RCC_SIZEOF_ENABLING_BITFIELD];

    switch (capabilities->family) {

    case STM32_FAMILY_F4:

        if (capabilities->f4.is_40x) {

            stm32f40x_usart_create_objects(obj, cm_state->svd_json,
                    periph_name);

        } else {
            assert(false);
        }

        state->reg.sr = state->u.f4.reg.sr;
        state->reg.dr = state->u.f4.reg.dr;
        state->reg.brr = state->u.f4.reg.brr;
        state->reg.cr1 = state->u.f4.reg.cr1;
        state->reg.cr2 = state->u.f4.reg.cr2;
        state->reg.cr3 = state->u.f4.reg.cr3;
        state->reg.gtpr = state->u.f4.reg.gtpr;

        // Register callbacks.
        peripheral_register_set_post_read(state->reg.dr,
                &stm32f4_usart_dr_post_read_callback);
        peripheral_register_set_post_write(state->reg.dr,
                &stm32f4_usart_dr_post_write_callback);
        peripheral_register_set_post_write(state->reg.cr1,
                &stm32f4_usart_cr1_post_write_callback);

        // char-device callbacks.
        if (state->chr) {
#if 1
            qemu_chr_fe_set_handlers(state->chr, stm32f4_usart_can_receive,
                    stm32f4_usart_receive, stm32f4_usart_event, obj, NULL, true);
#endif
        }

        switch (state->port_index) {

        case STM32_PORT_USART1:
        case STM32_PORT_USART6:
            snprintf(enabling_bit_name, sizeof(enabling_bit_name) - 1,
            DEVICE_PATH_STM32_RCC "/APB2ENR/USART%dEN",
                    1 + state->port_index - STM32_PORT_USART1);
            break;

        case STM32_PORT_USART2:
        case STM32_PORT_USART3:
        snprintf(enabling_bit_name, sizeof(enabling_bit_name) - 1,
        DEVICE_PATH_STM32_RCC "/APB1ENR/USART%dEN",
                1 + state->port_index - STM32_PORT_USART1);
        break;

        default:
            assert(false);
            break;
        }

        break;

    default:
        assert(false);
        break;
    }

    state->enabling_bit = OBJECT(cm_device_by_name(enabling_bit_name));

    peripheral_prepare_registers(obj);

    // ------------------------------------------------------------------------

    CharDriverState *chr = serial_hds[state->port_index];
    if (!chr) {
        char chardev_name[10];

        snprintf(chardev_name, ARRAY_SIZE(chardev_name) - 1, "serial%d",
                0 + state->port_index - STM32_PORT_USART1);
        chr = qemu_chr_new(chardev_name, "null");
        if (!(chr)) {
            hw_error("Can't assign serial port to %s.\n", periph_name);
        }
    }
    state->chr = chr;
}

static void stm32_usart_reset_callback(DeviceState *dev)
{
    qemu_log_function_name();

    STM32USARTState *state = STM32_USART_STATE(dev);

    // Call parent reset().
    cm_device_parent_reset(dev, TYPE_STM32_USART);

    if (state->chr) {
#if 1
        qemu_chr_fe_accept_input(state->chr);
#endif
    }

    const STM32Capabilities *capabilities =
    STM32_USART_STATE(state)->capabilities;
    assert(capabilities != NULL);

    switch (capabilities->family) {
    case STM32_FAMILY_F4:

        // TODO:
        // FIXME: We should certainly clear the interrupt state.
        // Don't know how to do that: implement cortexm_nvic_clear_pending ???

       //s->USART_SR_TXE = 1;
       //s->USART_SR_TC = 1;
       //s->USART_SR_RXNE = 0;
       //s->USART_SR_ORE = 0;
       peripheral_register_set_raw_value(state->reg.sr, 0x00000000);
       peripheral_register_set_raw_value(state->reg.brr, 0x00000000);
       peripheral_register_set_raw_value(state->reg.cr1, 0x00000000);
       peripheral_register_set_raw_value(state->reg.cr2, 0x00000000);
       peripheral_register_set_raw_value(state->reg.cr3, 0x00000000);
       peripheral_register_or_raw_value(state->reg.sr, USART_SR_TXE);
       peripheral_register_or_raw_value(state->reg.sr, USART_SR_TC);
       peripheral_register_or_raw_value(state->reg.cr1, USART_CR1_UE);
       peripheral_register_or_raw_value(state->reg.cr1, USART_CR1_TE);
       peripheral_register_or_raw_value(state->reg.cr1, USART_CR1_RE);

       // Do not initialize USART_DR - it is documented as undefined at reset
       // and does not behave like normal registers.
       //stm32_uart_USART_BRR_write(s, 0x00000000, true);
       //stm32_uart_USART_CR1_write(s, 0x00000000, true);
       //stm32_uart_USART_CR2_write(s, 0x00000000, true);
       //stm32_uart_USART_CR3_write(s, 0x00000000, true);

       stm32_uart_update_irq(dev);
        break;

    default:
        break;
    }

}

#if 0
static Property stm32_usart_properties[] = {
        DEFINE_PROP_CHR("chardev", STM32USARTState, chr),
        DEFINE_PROP_INT32_TYPE("port-index", STM32USARTState, port_index,
                STM32_USART_PORT_UNDEFINED, stm32_usart_index_t),
        DEFINE_PROP_NON_VOID_PTR("rcc", STM32USARTState, rcc, STM32RCCState *),
        DEFINE_PROP_NON_VOID_PTR("nvic", STM32USARTState,
                nvic, CortexMNVICState *),
        DEFINE_PROP_NON_VOID_PTR("capabilities", STM32USARTState,
                capabilities, const STM32Capabilities *),
    DEFINE_PROP_END_OF_LIST() };
#endif

static void stm32_usart_class_init_callback(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = stm32_usart_reset_callback;
    dc->realize = stm32_usart_realize_callback;

    //dc->props = stm32_usart_properties;

    // Reason: instance_init() method uses qemu_char_get_next_serial()
    // dc->cannot_instantiate_with_device_add_yet = true;

    PeripheralClass *per_class = PERIPHERAL_CLASS(klass);
    per_class->is_enabled = stm32_usart_is_enabled;
}

static const TypeInfo stm32_usart_type_info = {
    .name = TYPE_STM32_USART,
    .parent = TYPE_STM32_USART_PARENT,
    .instance_init = stm32_usart_instance_init_callback,
    .instance_size = sizeof(STM32USARTState),
    .class_init = stm32_usart_class_init_callback,
    .class_size = sizeof(STM32USARTClass)
/**/
};

static void stm32_usart_register_types(void)
{
    type_register_static(&stm32_usart_type_info);
}

type_init(stm32_usart_register_types);
