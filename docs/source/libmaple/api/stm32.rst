.. highlight:: c
.. _libmaple-stm32:

``stm32.h``
===========

General STM32-specific definitions.  This file is somewhat incomplete
(and LeafLabs-board specific) at the moment, but it will form the
future basis for MCU-related (rather than board-related, as this all
properly belongs in the Wirish layer) configuration information.

At the moment, it defines ``NR_INTERRUPTS`` appropriately based on
which of ``STM32_MEDIUM_DENSITY`` or ``STM32_HIGH_DENSITY`` is
defined.
