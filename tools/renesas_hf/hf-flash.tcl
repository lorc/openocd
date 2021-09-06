source [find mem_helper.tcl]

# Test function just to ensure that HF is accessible
proc read_flash_id { } {
    set backup_cmncr [mrw 0xee200000]

    mww 0xee20007C  0x80000263
    mww 0xee200000  0x81fff301
    mww 0xee200024  0x0
    mww 0xee200028  0x555
    mww 0xee20002C  0x0
    mww 0xee200064  0x5101
    mww 0xee200030  0xA2225408
    mww 0xee200040  0xF0000000
    mww 0xee200020  0x3

    mww 0xee20007C  0x80000263
    mww 0xee200000  0x81fff301
    mww 0xee200024  0x0
    mww 0xee200028  0x555
    mww 0xee20002C  0x0
    mww 0xee200064  0x5101
    mww 0xee200030  0xA2225408
    mww 0xee200040  0x98000000
    mww 0xee200020  0x3

    mww 0xee20007C  0x80000263
    mww 0xee200000  0x81fff301
    mww 0xee200024  0x00800000
    mww 0xee200028  0x0
    mww 0xee20002C  0x0
    mww 0xee200060  0xE
    mww 0xee200064  0x5101
    mww 0xee200030  0xA222D40C
    mww 0xee200020  0x5

    sleep 10
    set temp [mrw 0xee200038]
    echo [format "ID 1st: 0x%x (Manufacture)" [expr ($temp>>8) & 0xFF]]
    echo [format "ID 2nd: 0x%x (Device ID)"   [expr ($temp>>24)& 0xFF]]

    mww 0xee200000  $backup_cmncr
}

proc hf_execute_op { } "hf_execute_op64"

# Run on-board tool
proc hf_execute_op64 {  } {
    reg sp $::_hf_stack_addr
    reg pc $::_hf_code_addr
    resume
    wait_halt 20000

    set ret [get_reg x0]
    if {$ret != 0} {
	echo [format "HF: Error. Exit code is %X" $ret]
	exit 1
    }
}

# setup addreses
proc hf_setup { } {
    global _hf_code_addr
    global _hf_code_size
    global _hf_data_addr
    global _hf_data_size
    global _hf_stack_addr
    global _hf_stack_size
    # Configure code address
    set _hf_code_addr 0xE6328000

    # Configure code max size
    set _hf_code_size 0x2000

    # Configure data area
    set _hf_data_addr [expr 0xE632A000]

    # Configure data size
    set _hf_data_size 0x2000

    # Configure stack size
    set _hf_stack_size 0x100

    # Configure stack area
    set _hf_stack_addr [expr $_hf_data_addr + $_hf_data_size + $_hf_stack_size]

    echo [format "HF: Data:%x - %x Stack: %x - %x" $_hf_data_addr \
	      [expr $_hf_data_addr + $_hf_data_size] $_hf_stack_addr [expr $_hf_stack_addr - $_hf_stack_size]]

}

# Get TCL-frindly register value
proc get_reg {regname} {
    return [lindex [reg $regname] 2]
}

# Initialize on-board tool
proc hf_flash_init { } {

    hf_setup

    # Enable RPC PowerON & Clock
    mww 0xE6150900 [expr ~0x3F1E017]
    # Module STOP register, BIT17 RPC
    mww 0xE6150994 0x3F1E017

    mww 0xE6150900 [expr ~0x13]
    #0x13 =  80Mhz, 0x17=40Mhz
    mww 0xE6150238 0x13

    # Upload target flash tool
    load_image renesas_hf.bin $::_hf_code_addr bin

    hf_read_id
}

# Read info about flash
proc hf_read_id { } {
    # Base addr
    reg x0 0xEE200000
    # Cmd - CMD_READ_ID
    reg x1 0x0
    hf_execute_op
    set id_word [get_reg x1]
    echo [format "Hyper Flash ID: 0x%x" $id_word]
}

# Erase blocks
proc hf_erase {addr count} {
    echo [format "Hyper Flash: erasing 0x%x bytes at 0x%x" $count $addr]

    # Base addr
    reg x0 0xEE200000
    # Cmd - CMD_ERASE
    reg x1 0x1

    reg x2 $addr
    reg x3 $count

    hf_execute_op
}

proc _hf_load_partial {fname foffset address length } {
    # Load data from fname filename at foffset offset to
    # target at address. Load at most length bytes.
    load_image $fname [expr $address - $foffset] bin \
	$address $length
}

proc hf_write_file {fname addr} {
    set fstat [file stat $fname]
    set rem $fstat(size)
    set offs 0
    echo [format "Hyper Flash: Flashing $fname size %d at 0x%X" $fstat(size) $addr]
    while {$rem > 0} {
	echo [format "HF: Writing to addr 0x%06X remaining %d" $addr $rem]
	_hf_load_partial $fname $offs $::_hf_data_addr $::_hf_data_size

	# Base addr
	reg x0 0xEE200000
	# Cmd - CMD_WRITE
	reg x1 0x3

	reg x2 $addr
	reg x3 $::_hf_data_size
	reg x4 $::_hf_data_addr

	hf_execute_op

	set rem  [expr $rem  - $::_hf_data_size]
	set addr [expr $addr + $::_hf_data_size]
	set offs [expr $offs + $::_hf_data_size]
    }
}
