#!/bin/bash

echo "firmware path: $1"

ssh pi@cooking-mama 'mkdir -p ~/firmware_bas_niveau'
ssh pi@cooking-mama 'rm firmware_bas_niveau/*'
scp $1 pi@cooking-mama:~/firmware_bas_niveau/
ssh pi@cooking-mama 'st-flash --reset write ~/firmware_bas_niveau/firmware.bin 0x8000000'

