
# ======================================================================
#
# TCL script for the multiflow RIPPLE scheme
#
# ======================================================================
# Author:           Tianji Li
# Date:             30/03/2009
# Organization:     Hamilton Institute, NUIM, Ireland
# ======================================================================


# set numflow    { 1  1  1 2 2 2 3 3 3 }
set numflow    { 10 10 10 20 20 20 30 30 30 }
# set BER        { 0.000001 }
set BER        { 0.00001 0.000001 }
set rate 216
set basicrate 54
set stop 10


        set f_throughput  [open "results.web.txt" "a"]
        puts $f_throughput " "
        puts $f_throughput " "
        puts $f_throughput " "
        puts $f_throughput "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
        puts $f_throughput "AFR WEB: $numflow"
        puts $f_throughput "AFR WEB: $BER"
        set aaa "+DATE(month/day/year): %m/%d/%Y, TIME: %H:%M:%S"
        set systemtime       "[exec date $aaa]"
        puts $f_throughput "$systemtime"
        puts $f_throughput "rate: $rate Mbps"
        puts $f_throughput "run: $stop seconds"
        puts $f_throughput "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
        puts $f_throughput " "
        puts $f_throughput " "
        puts $f_throughput " "
        close $f_throughput 
 for {set i 0} {$i < [llength $BER] } {incr i} {
         for {set j 0} {$j < [llength $numflow] } {incr j} {
puts [lindex $numflow $j] 
puts [lindex $BER $i]
#                 exec ns afr.multiflow.tcl 10 6 6  [lindex $numflow $j] [lindex $BER $i] >> "auto-result.txt" 2> /dev/null
                exec ns afr.web.tcl $stop $rate $basicrate  [lindex $numflow $j] [lindex $BER $i] >> "auto-result.txt" 2> /dev/null
        }
        set f_throughput  [open "results.web.txt" "a"]
        puts $f_throughput ""
        puts $f_throughput ""
        close $f_throughput
 }

