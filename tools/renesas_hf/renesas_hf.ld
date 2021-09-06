ENTRY(_entry)
PHDRS { text PT_LOAD ; }
SECTIONS
{
	.text : {
		KEEP(*(.entry))
		KEEP(*(.text.))
		} : text
}
