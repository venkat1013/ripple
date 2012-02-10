
# ======================================================================
#
# TCL script for the multiflow RIPPLE scheme
#
# ======================================================================
# Author:           Tianji Li
# Date:             30/03/2009
# Organization:     Hamilton Institute, NUIM, Ireland
# ======================================================================


set numflow    { 2 }
set hops    { 2 3 4 5 6 7 }
# set numflow    { 10 10 10 20 20 20 30 30 30 }
# set BER        { 0.00001 }
set BER        { 0.000001 }
set rate 216
set basicrate 54
set stop 10

        set f_throughput  [open "results.7hops.txt" "a"]
        puts $f_throughput " "
        puts $f_throughput " "
        puts $f_throughput " "
        puts $f_throughput "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
        puts $f_throughput "AFR 7HOPS: $numflow"
        puts $f_throughput "AFR 7HOPS: $BER"
        puts $f_throughput "AFR 7HOPS: $hops"
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
         for {set k 0} {$k < [llength $hops] } {incr k} {
            exec ns afr.7hops.tcl $stop $rate $basicrate  [lindex $numflow $j] [lindex $BER $i] [lindex $hops $k] >> "auto-result.txt" 2>> "auto-result.txt"
         }
    }
 }
