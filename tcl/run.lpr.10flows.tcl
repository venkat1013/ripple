
# ======================================================================
#
# TCL script for the multiflow RIPPLE scheme
#
# ======================================================================
# Author:           Tianji Li
# Date:             30/03/2009
# Organization:     Hamilton Institute, NUIM, Ireland
# ======================================================================


# set numflow    { 1 2 3 4 5 6 7 8 9 10  }
set numflow    { 8 9 10  }
set BER        { 0.000001 }

        set f_throughput  [open "results.multiflow.txt" "a"]
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

set f_10  [open "throughput.10flows.txt" "a"]
puts $f_10 ""
puts $f_10 ""
puts $f_10 "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
puts $f_10 "LPR $numflow"
puts $f_10 "BER $BER"
puts $f_10 "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
close $f_10

# set f_11  [open "fairness.10flows.txt" "a"]
# puts $f_11 "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
# puts $f_11 "LPR $numflow"
# puts $f_11 "BER $BER"
# puts $f_11 "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
for {set i 0} {$i < [llength $BER] } {incr i} {
         for {set j 0} {$j < [llength $numflow] } {incr j} {
                puts [lindex $numflow $j] 
                puts [lindex $BER $i]
#                 exec ns lpr.10flows.tcl 10 6 6 [lindex $numflow $j] [lindex $BER $i] >> "auto-result.txt" 2> /dev/null
                exec ns lpr.10flows.tcl 10 216 54  [lindex $numflow $j] [lindex $BER $i] >> "auto-result.txt" 2> /dev/null
        }
}
# close $f_11

