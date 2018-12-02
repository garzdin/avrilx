// Copyright 2012 Olivier Gillet.
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
//
// -----------------------------------------------------------------------------
//
// Interrupt handlers for USART.

#include <avrlibx/io/usart.h>

#include <avr/interrupt.h>

using namespace avrlibx;

// USART C0 interrupt handlers

#ifdef USARTC0_RXC_vect

void (*USARTWrapper<PortC, 0>::rx_handler_)() = 0;

ISR(USARTC0_RXC_vect) {
  USARTWrapper<PortC, 0>::rx_handler();
}

#endif

#ifdef USARTC0_TXC_vect

void (*USARTWrapper<PortC, 0>::tx_handler_)() = 0;

ISR(USARTC0_TXC_vect) {
  USARTWrapper<PortC, 0>::tx_handler();
}

#endif

#ifdef USARTC0_DRE_vect

void (*USARTWrapper<PortC, 0>::dre_handler_)() = 0;

ISR(USARTC0_DRE_vect) {
  USARTWrapper<PortC, 0>::dre_handler();
}

#endif

// USART C1 interrupt handlers

#ifdef USARTC1_RXC_vect

void (*USARTWrapper<PortC, 1>::rx_handler_)() = 0;

ISR(USARTC1_RXC_vect) {
  USARTWrapper<PortC, 1>::rx_handler();
}

#endif

#ifdef USARTC1_TXC_vect

void (*USARTWrapper<PortC, 1>::tx_handler_)() = 0;

ISR(USARTC1_TXC_vect) {
  USARTWrapper<PortC, 1>::tx_handler();
}

#endif

#ifdef USARTC1_DRE_vect

void (*USARTWrapper<PortC, 1>::dre_handler_)() = 0;

ISR(USARTC1_DRE_vect) {
  USARTWrapper<PortC, 1>::dre_handler();
}

#endif

// USART D0 interrupt handlers

#ifdef USARTD0_RXC_vect

void (*USARTWrapper<PortD, 0>::rx_handler_)() = 0;

ISR(USARTD0_RXC_vect) {
  USARTWrapper<PortD, 0>::rx_handler();
}

#endif

#ifdef USARTD0_TXC_vect

void (*USARTWrapper<PortD, 0>::tx_handler_)() = 0;

ISR(USARTD0_TXC_vect) {
  USARTWrapper<PortD, 0>::tx_handler();
}

#endif

#ifdef USARTD0_DRE_vect

void (*USARTWrapper<PortD, 0>::dre_handler_)() = 0;

ISR(USARTD0_DRE_vect) {
  USARTWrapper<PortD, 0>::dre_handler();
}

#endif

// USART D1 interrupt handlers

#ifdef USARTD1_RXC_vect

void (*USARTWrapper<PortD, 1>::rx_handler_)() = 0;

ISR(USARTD1_RXC_vect) {
  USARTWrapper<PortD, 1>::rx_handler();
}

#endif

#ifdef USARTD1_TXC_vect

void (*USARTWrapper<PortD, 1>::tx_handler_)() = 0;

ISR(USARTD1_TXC_vect) {
  USARTWrapper<PortD, 1>::tx_handler();
}

#endif

#ifdef USARTD1_DRE_vect

void (*USARTWrapper<PortD, 1>::dre_handler_)() = 0;

ISR(USARTD1_DRE_vect) {
  USARTWrapper<PortD, 1>::dre_handler();
}

#endif

// USART E0 interrupt handlers

#ifdef USARTE0_RXC_vect

void (*USARTWrapper<PortE, 0>::rx_handler_)() = 0;

ISR(USARTE0_RXC_vect) {
  USARTWrapper<PortE, 0>::rx_handler();
}

#endif

#ifdef USARTE0_TXC_vect

void (*USARTWrapper<PortE, 0>::tx_handler_)() = 0;

ISR(USARTE0_TXC_vect) {
  USARTWrapper<PortE, 0>::tx_handler();
}

#endif

#ifdef USARTE0_DRE_vect

void (*USARTWrapper<PortE, 0>::dre_handler_)() = 0;

ISR(USARTE0_DRE_vect) {
  USARTWrapper<PortE, 0>::dre_handler();
}

#endif

#ifdef USARTE1
  // USART E1 interrupt handlers

  #ifdef USARTE1_RXC_vect

  void (*USARTWrapper<PortE, 1>::rx_handler_)() = 0;

  ISR(USARTE1_RXC_vect) {
    USARTWrapper<PortE, 1>::rx_handler();
  }

  #endif

  #ifdef USARTE1_TXC_vect

  void (*USARTWrapper<PortE, 1>::tx_handler_)() = 0;

  ISR(USARTE1_TXC_vect) {
    USARTWrapper<PortE, 1>::tx_handler();
  }

  #endif

  #ifdef USARTE1_DRE_vect

  void (*USARTWrapper<PortE, 1>::dre_handler_)() = 0;

  ISR(USARTE1_DRE_vect) {
    USARTWrapper<PortE, 1>::dre_handler();
  }

  #endif
#endif

#ifdef USARTF0
  // USART F0 interrupt handlers

  #ifdef USARTF0_RXC_vect

  void (*USARTWrapper<PortF, 0>::rx_handler_)() = 0;

  ISR(USARTF0_RXC_vect) {
    USARTWrapper<PortF, 0>::rx_handler();
  }

  #endif

  #ifdef USARTF0_TXC_vect

  void (*USARTWrapper<PortF, 0>::tx_handler_)() = 0;

  ISR(USARTF0_TXC_vect) {
    USARTWrapper<PortF, 0>::tx_handler();
  }

  #endif

  #ifdef USARTF0_DRE_vect

  void (*USARTWrapper<PortF, 0>::dre_handler_)() = 0;

  ISR(USARTF0_DRE_vect) {
    USARTWrapper<PortF, 0>::dre_handler();
  }

  #endif
#endif

#ifdef USARTF1
  // USART F1 interrupt handlers

  #ifdef USARTF1_RXC_vect

  void (*USARTWrapper<PortF, 1>::rx_handler_)() = 0;

  ISR(USARTF1_RXC_vect) {
    USARTWrapper<PortF, 1>::rx_handler();
  }

  #endif

  #ifdef USARTF1_TXC_vect

  void (*USARTWrapper<PortF, 1>::tx_handler_)() = 0;

  ISR(USARTF1_TXC_vect) {
    USARTWrapper<PortF, 1>::tx_handler();
  }

  #endif

  #ifdef USARTF1_DRE_vect

  void (*USARTWrapper<PortF, 1>::dre_handler_)() = 0;

  ISR(USARTF1_DRE_vect) {
    USARTWrapper<PortF, 1>::dre_handler();
  }

  #endif
#endif
