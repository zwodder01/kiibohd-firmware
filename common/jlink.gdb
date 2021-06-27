# print demangled symbols
set print asm-demangle on

# set backtrace limit to not have infinite backtrace loops
set backtrace limit 32

# detect unhandled exceptions, hard faults and panics
break DefaultHandler
break HardFault
break rust_begin_unwind

# Update stack pointer to point to top of ram
# TODO (HaaTa): Figure out how to support flip-link
# TODO (HaaTa): Figure out how to parameterize this for different RAM sizes
set $sp = 0x20020000
# Setup VTOR
set *(0xE000ED08 as *mut u32) = 0x406000

# Continue running
#continue
