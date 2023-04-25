#!/bin/bash

# hostname ou adresse ip de Cooking Mama
COOKING_MAMA=cooking-mama

echo "firmware path: $1"

ssh pi@$COOKING_MAMA 'mkdir -p ~/firmware_bas_niveau'
ssh pi@$COOKING_MAMA 'rm firmware_bas_niveau/*'
scp $1 pi@$COOKING_MAMA:~/firmware_bas_niveau/
ssh pi@$COOKING_MAMA 'st-flash --reset write ~/firmware_bas_niveau/firmware.bin 0x8000000'

