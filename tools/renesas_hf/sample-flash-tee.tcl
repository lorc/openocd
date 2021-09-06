source [find "target/renesas_rcar_gen3.cfg"]
source "hf-flash.tcl"

# This demo script will erase OP-TEE on Hyperfalsh and then write a
# new one from "tee-raw.bin"

init
reset halt
targets r8a77950.a57.0

hf_flash_init
hf_erase 0x200000 0x300000
hf_write_file "tee-raw.bin" 0x200000

reset run

exit
