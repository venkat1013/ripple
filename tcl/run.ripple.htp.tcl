
# ======================================================================
#
# TCL script for the multiflow RIPPLE scheme
#
# ======================================================================
# Author:           Tianji Li
# Date:             30/03/2009
# Organization:     Hamilton Institute, NUIM, Ireland
# ======================================================================

set numflow    { 5 6 7 8 9 }
# set numflow    { 1 2 3 4 5 6 7 8 9 10  }
set BER        { 0.000001 }
        set f_throughput  [open "results.htp.txt" "a"]
        puts $f_throughput " "
        puts $f_throughput " "
        puts $f_throughput " "
        puts $f_throughput "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
        puts $f_throughput "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
        puts $f_throughput "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
        puts $f_throughput " "
        puts $f_throughput " "
        puts $f_throughput " "
        close $f_throughput 


set f_10  [open "throughput.htp.txt" "a"]
puts $f_10 ""
puts $f_10 ""
puts $f_10 "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
puts $f_10 "RIPPLE $numflow"
puts $f_10 "BER $BER"
puts $f_10 "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
close $f_10



 for {set i 0} {$i < [llength $BER] } {incr i} {
         for {set j 0} {$j < [llength $numflow] } {incr j} {
puts [lindex $numflow $j] 
puts [lindex $BER $i]
                exec ns ripple.htp.tcl 10 216 54  [lindex $numflow $j] [lindex $BER $i] >> "auto-result.txt" 2> /dev/null
        }
 }

