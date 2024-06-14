#!/bin/bash

ip="192.168.0.11"
hostName="ubuntu"

filePathWindows="./NUCLEO_F411RE/"
folderPathLinux="~/stm_binary"
fileName="dc_motor_controll_new.bin"

scp -i ~/.ssh/raspberry "${filePathWindows}${fileName}" "${hostName}@${ip}:${folderPathLinux}"

command="st-flash write ${folderPathLinux}/${fileName} 0x8000000"

ssh -i ~/.ssh/test "${hostName}@${ip}" "${command}"