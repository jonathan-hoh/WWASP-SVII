#!/bin/bash

scp * pi@10.206.160.116:~/SVII_R8_REALTIME

ssh pi@10.206.160.116 "cd ~/SVII_R8_REALTIME; \
	 	       ./MAKE"

