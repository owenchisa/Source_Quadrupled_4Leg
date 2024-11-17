#!/usr/bin/env python3
import os
import sys
import time
import tm1640 as tm

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

print('''
**********************************************************
********************function:dot matrix display routine********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program. If the program cannot be closed, please try again.
----------------------------------------------------------
''')

# the dot matrix module should be connected to IO7 and IO8 interface on expansion board


if __name__ == '__main__':
    # display 'Hello'
    tm.display_buf = (0x7f, 0x08, 0x7f, 0x00, 0x7c, 0x54, 0x5c, 0x00,
                       0x7f, 0x40, 0x00,0x7f, 0x40, 0x38, 0x44, 0x38)
    tm.update_display() # update display

    time.sleep(5) # delay
    
    tm.display_buf = [0] * 16 # Cache zeroing 
    tm.update_display() # update display
