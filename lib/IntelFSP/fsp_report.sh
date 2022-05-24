#!/bin/bash

python3 SplitFspBin.py info -f TigerLake/FspRel.bin > TigerLake/FspInfo.txt
python3 SplitFspBin.py info -f QEMU/FspRel.bin > QEMU/FspInfo.txt
