
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
set numflow    { 1 1 1  }
set BER        { 0.00000 }
        set f_throughput  [open "results.exor.multiflow.txt" "a"]
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
 for {set i 0} {$i < [llength $BER] } {incr i} {
         for {set j 0} {$j < [llength $numflow] } {incr j} {
puts [lindex $numflow $j] 
puts [lindex $BER $i]
                exec ns exor.multiflow.tcl 10 216 54  [lindex $numflow $j] [lindex $BER $i] >> "auto-result.txt" 2> /dev/null
#                 exec ns exor.multiflow.tcl 10 6 6  [lindex $numflow $j] [lindex $BER $i] >> "auto-result.txt" 2> /dev/null
        }
 }

