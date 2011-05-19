.. highlight:: c
.. _libmaple-util:

``util.h``
==========

.. TODO [0.2.0?] clean this up; Sphinx/Breathe aren't really ready yet though

Miscellaneous utility macros and procedures.

Bit Manipulation
----------------

::

    #define BIT(shift)                     (1UL << (shift))
    #define BIT_MASK_SHIFT(mask, shift)    ((mask) << (shift))
    /** Gets bits m to n of x */
    #define GET_BITS(x, m, n) ((((uint32)x) << (31 - (n))) >> ((31 - (n)) + (m)))
    #define IS_POWER_OF_TWO(v)  (v && !(v & (v - 1)))

Failure Routines
----------------

.. doxygenfunction:: throb
