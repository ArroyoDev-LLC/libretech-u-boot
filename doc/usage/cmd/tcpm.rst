.. SPDX-License-Identifier: GPL-2.0+:

.. index::
   single: tcpm (command)

tcpm command
============

Synopsis
--------

::

    tcpm dev [devnum]
    tcpm info
    tcpm list
    tcpm dp
    tcpm dp enter
    tcpm dp exit [-f]

Description
-----------

The tcpm command is used to control USB-PD controllers, also known as TypeC Port Manager (TCPM).

The 'tcpm dev' command shows or set current TCPM device.

    devnum
        device number to change

The 'tcpm info' command displays the current state of the device. The
TCPM state is always printed; the orientation, PD revision, roles and the
negotiated voltage and current follow only while a partner is attached.

The 'tcpm list' command displays the list available devices.

The 'tcpm dp' command discovers the partner's DisplayPort alternate mode on
the current device. 'tcpm dp enter' additionally enters it, with this port
as the DisplayPort source; 'tcpm dp exit' leaves it. '-f' sends the exit
even when this port holds no record of having entered the mode, for a
partner left in it by a previous boot.

Examples
--------

The 'tcpm info' command displays device's status. With a partner attached
and a PD contract in place:
::

    => tcpm info
    TCPM State:  SNK_READY
    Orientation: reverse
    PD Revision: rev3
    Power Role:  sink
    Data Role:   device
    Voltage:      5.000 V
    Current:      0.000 A

With nothing attached only the state is shown:
::

    => tcpm info
    TCPM State:  TOGGLING

The current device can be shown or set via 'tcpm dev' command:
::

    => tcpm dev
    TCPM device is not set!
    => tcpm dev 0
    dev: 0 @ usb-typec@22
    => tcpm dev
    dev: 0 @ usb-typec@22

The list of available devices can be shown via 'tcpm list' command:
::

    => tcpm list
    | ID | Name                            | Parent name         | Parent uclass @ seq
    |  0 | usb-typec@22                    | i2c@ff160000        | i2c @ 7 | status: 0
    |  1 | usb-typec@22                    | i2c@ff3d0000        | i2c @ 4 | status: 0

Configuration
-------------

The tcpm command is only available if CONFIG_CMD_TCPM=y.
