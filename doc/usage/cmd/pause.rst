.. SPDX-License-Identifier: GPL-2.0-or-later:

.. index::
   single: pause (command)

pause command
=============

Synopsis
--------

::

    pause [prompt]


Description
-----------

The pause command delays execution waiting for any user input.

It can accept a single parameter to change the prompt message.

Return value
------------

The return value $? is 0 (true) for any key except Ctrl-C, and 1 (false) when
Ctrl-C is pressed. This makes it usable as an acknowledgement prompt::

    if pause "Press Control+C to cancel or any key to continue."; then
        run risky_command
    fi

Note that a ';' chain ignores the return value, as it does for every other
command: in ``pause "..."; run risky_command`` the second command always runs.

Examples
--------

Using with the default prompt:

::

    => pause
    Press any key to continue...


Using with a custom prompt:

::

    => pause 'Prompt for pause...'
    Prompt for pause...

Note that complex prompts require proper quoting:

::

    => pause Prompt for pause...
    pause - delay until user input
    
    Usage:
    pause [prompt] - Wait until users presses any key. [prompt] can be used to customize the message.
