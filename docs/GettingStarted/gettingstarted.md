# Overview

Simple Open EtherCAT MainDevice \(SOEM\) is an open source EtherCAT MainDevice stack used to write custom EtherCAT MainDevice applications. It provides a small footprint and is suited for embedded systems. For more information on SOEM, see [https://openethercatsociety.github.io/doc/soem/](https://openethercatsociety.github.io/doc/soem/).

The SOEM is integrated in MCUXpresso SDK version 2.12.1. A hardware abstraction layer is added to support the ENET and NETC Ethernet interface in the i.MX RT series. To support FreeRTOS and Baremetal, OS abstraction layers \(OSAL\) are added.


# Hardware requirements

The SOEM applications in the MCUXpresso SDK require the following hardware components:

-   General Purpose Timer \(GPT\) for scheduling tasks or as a system clock
-   ENET and NETC Ethernet interface or two for cable redundancy

The SOEM applications support the following EVK boards in the MCUXpresso SDK:

-   evkmimxrt1020
-   evkmimxrt1040
-   evkbimxrt1050
-   evkbmimxrt1060
-   evkmimxrt1060
-   evkmimxrt1064
-   evkmimxrt1160
-   evkmimxrt1170
-   evkmimxrt1180
-   imx943evk


# SOEM middleware in MCUXpresso SDK

In the MCUXpresso SDK, the SOEM middleware is in the directory, `middleware/soem`. To support the ENET and NETC Ethernet driver \(`fsl_enet and fsl_netc`\) in the SDK, a hardware abstraction layer in the SOEM is integrated, which is located in the directory, `middleware/soem/oshw/mcux-sdk/`. Therefore, the SOEM middleware can support all i.MX RT MCUs integrated the ENET and NETC Ethernet interface.

For the cases where real-time high performance is a critical requirement, Baremetal is used. The OS abstraction layer \(OSAL\) for Baremetal is located in the directory, `middleware/soem/osal/baremetal/`. For the cases where multitasking is critical, FreeRTOS is used. The OS abstraction layer \(OSAL\) for FreeRTOS is located in the directory, `middleware/soem/osal/freertos/`.


# SOEM example applications in MCUXpresso SDK

There are 3 examples in the MCUXpresso SDK based on the SOEM middleware.

The first of which control a simulated linear guide system consisting of three EtherCAT devices, a linear guide, two limit switches, and a stepper motor.

-   EK1100, a coupler, is one of the EtherCAT devices that connects EVK boards to the other EtherCAT IO devices.
-   EL2008 supports 8 output IO ports connected to the stepper motor.
-   EL1018, supports 8 input IO ports connected to the two limit switches.

The task of the examples controls the linear rail system using two output and two input remote IO signals:

-   Output pulse: Output the pulse with a 200 μs period and 50 % duty cycle via the EL2008 device. It is used to drive the stepper motor.
-   Motor direction: Output via the EL2008 device to control the stepper motor rotation direction.
-   Two limit signals: Two input signals via the EL1018 device connected to limit switches to indicate the status of the switches. A high level indicates that the limit switch is closed.

The task of the examples always outputs the Output pulse and Motor direction signals to drive the stepper motor. The Motor direction signal changes when one of the limit switches is closed.

The seconed examples controls servo motor:

-   soem running on RT series board
-   Using commercial servo motor as EtherCAT subdevice controller

The third examples  controls servo motor which running on i.MX RT1180:

-   soem running on RT series board
-   Using i.MX RT1180 as EtherCAT subdevice controller


