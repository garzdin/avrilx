// Copyright 2011 Olivier Gillet.
//
// Author: Olivier Gillet (ol.gillet@gmail.com)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef AVRLIBX_IO_USART_H_
#define AVRLIBX_IO_USART_H_

#include <avr/io.h>

#include <avrlibx/avrlibx.h>
#include <avrlibx/io/gpio.h>
#include <avrlibx/io/ring_buffer.h>

namespace avrlibx {

enum RxMode {
  RX_DISABLED,
  RX_ENABLED
};

enum TxMode {
  TX_DISABLED,
  TX_ENABLED
};

enum USARTState {
  USART_STATE_READY,
  USART_STATE_TRANSMITTING,
  USART_STATE_RECEIVING,
};

enum USARTError {
  USART_ERROR_NONE = 0,
  USART_ERROR_FRAME = 1,
  USART_ERROR_BUFFER_OVERFLOW = 2,
  USART_ERROR_PARITY = 3,
  USART_ERROR_UNEXPECTED_STATE = 4
};

template<typename Port, uint8_t index> struct USARTWrapper { };

#define WRAP_USART(port, index) \
template<> \
struct USARTWrapper<Port ## port, index> { \
  static inline USART_t& usart() { \
    return USART ## port ## index; \
  } \
  static volatile inline uint8_t data() { \
    return USART ## port ## index ## _DATA; \
  } \
  static inline void set_data(uint16_t value) { \
    USART ## port ## index ## _DATA = value; \
  } \
  static inline uint8_t readable() { \
    return USART ## port ## index ## _STATUS & USART_RXCIF_bm; \
  } \
  static inline uint8_t writable_buffer() { \
    return USART ## port ## index ## _STATUS & USART_DREIF_bm; \
  } \
  static inline uint8_t writable() { \
    return USART ## port ## index ## _STATUS & USART_TXCIF_bm; \
  } \
  static inline uint8_t dma_rx_trigger() { \
    return DMA_CH_TRIGSRC_USART ## port ## index ## _RXC_gc; \
  } \
  static inline uint8_t dma_tx_trigger() { \
    return DMA_CH_TRIGSRC_USART ## port ## index ## _DRE_gc; \
  } \
  static inline volatile void* dma_data() { \
    return &(USART ## port ## index ## _DATA); \
  } \
  static inline void rx_handler() { \
    if (rx_handler_) { \
      (*rx_handler_)(); \
    } \
  } \
  static inline void tx_handler() { \
    if (tx_handler_) { \
      (*tx_handler_)(); \
    } \
  } \
  static inline void dre_handler() { \
    if (dre_handler_) { \
      (*dre_handler_)(); \
    } \
  } \
  static inline void set_rx_interrupt_handler(void (*handler)()) { \
    rx_handler_ = handler; \
  } \
  static inline void set_tx_interrupt_handler(void (*handler)()) { \
    tx_handler_ = handler; \
  } \
  static inline void set_dre_interrupt_handler(void (*handler)()) { \
    dre_handler_ = handler; \
  } \
  static inline void (*rx_handler_)(); \
  static inline void (*tx_handler_)(); \
  static inline void (*dre_handler_)(); \
};

WRAP_USART(C, 0)
WRAP_USART(C, 1)
WRAP_USART(D, 0)
WRAP_USART(D, 1)
WRAP_USART(E, 0)
#ifdef USARTE1
  WRAP_USART(E, 1)
#endif
#ifdef USARTF0
  WRAP_USART(F, 0)
#endif
#ifdef USARTF1
  WRAP_USART(F, 1)
#endif

template<typename Port, uint8_t size = 256>
class USARTOutputBufferSpecs {
 public:
  USARTOutputBufferSpecs() { }
  enum {
    buffer_size = size,
    data_size = 8
  };
  typedef uint8_t Value;
};

template<typename Port, uint8_t size = 256>
class USARTInputBufferSpecs {
 public:
  USARTInputBufferSpecs() { }
  enum {
    buffer_size = size,
    data_size = 8
  };
  typedef uint8_t Value;
};

template<
    typename Port,
    uint8_t index,
    uint32_t baud_rate,
    RxMode rx_mode,
    TxMode tx_mode,
    uint8_t input_buffer_size = 256,
    uint8_t output_buffer_size = 256,
    uint8_t rx_int_lvl = 1,
    uint8_t tx_int_lvl = 1,
    uint8_t dre_int_lvl = 1>
struct Usart {
  typedef USARTWrapper<Port, index> USART;

  static inline void Init() {
    Init<baud_rate, rx_int_lvl, tx_int_lvl, dre_int_lvl>();
  }

  template<uint32_t new_baud_rate, uint8_t new_rx_int_lvl, uint8_t new_tx_int_lvl, uint8_t new_dre_int_lvl>
  static inline void Init() {
    uint16_t prescaler = F_CPU / (16 * baud_rate) - 1;
    USART::usart().BAUDCTRLA = prescaler & 0xff;
    USART::usart().BAUDCTRLB = (prescaler >> 8) & 0xf;
    uint8_t control_a;
    uint8_t control_b;
    USART::set_rx_interrupt_handler(&RXInterruptHandler);
    USART::set_tx_interrupt_handler(&TXInterruptHandler);
    USART::set_dre_interrupt_handler(&DREInterruptHandler);
    if (rx_mode != RX_DISABLED) {
      control_a |= (rx_int_lvl << 4);
      control_b |= USART_RXEN_bm;
    }
    if (tx_mode != TX_DISABLED) {
      if (index == 0) {
        Gpio<Port, 3>::set_direction(OUTPUT);
      } else {
        Gpio<Port, 7>::set_direction(OUTPUT);
      }
      control_a |= (tx_int_lvl << 2);
      control_a |= (dre_int_lvl << 0);
      control_b |= USART_TXEN_bm;
    }

    USART::usart().CTRLA = control_a;
    USART::usart().CTRLB = control_b;

    error_ = USART_ERROR_NONE;
  }

  static uint8_t Wait() {
    while (state_ != USART_STATE_READY) { }
    return error_;
  }

  static void Done() {
    USART::usart().CTRLA = 0;
    USART::set_rx_interrupt_handler(NULL);
    USART::set_tx_interrupt_handler(NULL);
    USART::set_dre_interrupt_handler(NULL);
  }

  static inline void Write(uint8_t v) {
    state_ = USART_STATE_TRANSMITTING;
    error_ = USART_ERROR_NONE;

    Output::Write(v);
  }
  static inline uint8_t writable() { return Output::writable(); }
  static inline uint8_t NonBlockingWrite(uint8_t v) {
    uint8_t res = Output::NonBlockingWrite(v);

    state_ = USART_STATE_TRANSMITTING;
    error_ = USART_ERROR_NONE;

    return res;
  }
  static inline void Overwrite(uint8_t v) { Output::Overwrite(v); }

  static inline uint8_t Read() {
    uint8_t res = Input::Read();

    state_ = USART_STATE_RECEIVING;
    error_ = USART_ERROR_NONE;

    return res;
  }
  static inline uint8_t readable() { return Input::readable(); }
  static inline int16_t NonBlockingRead() {
    uint8_t res = Input::NonBlockingRead();

    state_ = USART_STATE_RECEIVING;
    error_ = USART_ERROR_NONE;

    return res;
  }
  static inline uint8_t ImmediateRead() { return Input::ImmediateRead(); }

  static inline void FlushInputBuffer() { Input::Flush(); }
  static inline void FlushOutputBuffer() { Output::Flush(); }

  static inline uint8_t dma_rx_trigger() { return USART::dma_rx_trigger(); }
  static inline uint8_t dma_tx_trigger() { return USART::dma_rx_trigger(); }
  static inline volatile void* dma_data() { return USART::dma_data(); }

  private:
   static void RXInterruptHandler() {
     uint8_t current_status = USART::usart().STATUS;
     if (current_status & USART_FERR_bm) {
       error_ = USART_ERROR_FRAME;
       state_ = USART_STATE_READY;
     } else if (current_status & USART_PERR_bm) {
       error_ = USART_ERROR_PARITY;
       state_ = USART_STATE_READY;
     } else if (current_status & USART_BUFOVF_bm) {
       error_ = USART_ERROR_BUFFER_OVERFLOW;
       state_ = USART_STATE_READY;
     } else {
       if (USART::readable()) {
         Input::Write(USART::data());
       }

       state_ = USART_STATE_READY;
     }
   }

   static void TXInterruptHandler() {
     state_ = USART_STATE_READY;
   }

   static void DREInterruptHandler() {
     if (USART::writable_buffer()) {
       USART::set_data(Output::ImmediateRead());
     }

     state_ = USART_STATE_READY;
   }

  public:
   typedef RingBuffer<USARTInputBufferSpecs<Port, input_buffer_size> > Input;
   typedef RingBuffer<USARTOutputBufferSpecs<Port, output_buffer_size> > Output;

  private:
   static volatile uint8_t state_;
   static volatile uint8_t error_;

   typedef uint8_t Value;
};

/* static */

template<typename P, uint8_t i, uint32_t br, RxMode rm, TxMode tm, uint8_t ibs, uint8_t obs, uint8_t ril, uint8_t til, uint8_t dil>
volatile uint8_t Usart<P, i, br, rm, tm, ibs, obs, ril, til, dil>::state_;

/* static */
template<typename P, uint8_t i, uint32_t br, RxMode rm, TxMode tm, uint8_t ibs, uint8_t obs, uint8_t ril, uint8_t til, uint8_t dil>
volatile uint8_t Usart<P, i, br, rm, tm, ibs, obs, ril, til, dil>::error_;

}  // namespace avrlibx

#endif   // AVRLIBX_IO_USART_H_
