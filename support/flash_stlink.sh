#!/bin/bash

serials=$(st-info --serial)

if [[ -z "$serials" ]]; then
   echo "No connected ST-Link."
   exit 1
fi

echo "$serials"
count=$(echo "$serials" | wc -l)

echo "$count"

if [[ "$count" -eq 1 ]]; then
   serial="$serials"
else
   echo "List of ST-Links: "
   select serial in $serials; do
      if [[ -n "$serial" ]]; then
         break
      else
         echo "Error. Try again."
      fi
   done
fi

openocd -f /usr/local/share/openocd/scripts/interface/st-link.cfg \
        -c "hla_serial $serial" \
        -f /usr/local/share/openocd/scripts/target/stm32f1x.cfg \
        -c "program out/target.elf verify reset exit"