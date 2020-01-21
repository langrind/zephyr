.. _ada1372_regs:

ADAU1372 Registers
##################

Overview
********
Stub of app to read and write registers of ADAU1372 Codec via I2C.

Building and Running
********************

cd apps/adau1372_regs && west build -b mimxrt1064_evk .


Sample Output
=============

.. code-block:: console

*** Booting Zephyr OS build zephyr-v2.1.0-1217-gb581d8ccd718  ***
Hello World! mimxrt1064_evk


uart:~$ i2c rd0
read 0 from ADI reg 0x00
i2c_read_test_1() device succeeded
uart:~$ i2c rd11
read 16 from ADI reg 0x11
i2c_read_test_2() device succeeded
uart:~$ 

