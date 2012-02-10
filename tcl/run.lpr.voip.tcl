
# ======================================================================
#
# TCL script for the multiflow RIPPLE scheme
#
# ======================================================================
# Author:           Tianji Li
# Date:             30/03/2009
# Organization:     Hamilton Institute, NUIM, Ireland
# ======================================================================


# set numflow    { 20 20 20 30 30 30 }
set numflow    { 10 10 10 20 20 20 30 30 30 }
set BER        { 0.000001}
# set BER        { 0.000001 }
set rate 6
set basicrate 6
set stop 10

        set f_throughput  [open "results.voip.txt" "a"]
        puts $f_throughput " "
        puts $f_throughput " "
        puts $f_throughput " "
        puts $f_throughput "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
        puts $f_throughput "LPR VOIP: $numflow"
        puts $f_throughput "LPR VOIP: $BER"
        set aaa "+DATE(month/day/year): %m/%d/%Y, TIME: %H:%M:%S"
        set systemtime       "[exec date $aaa]"
        puts $f_throughput "$systemtime"
        puts $f_throughput "rate: $rate Mbps"
        puts $f_throughput "run: $stop seconds"
        puts $f_throughput "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
        puts $f_throughput " "
        puts $f_throughput " "
        puts $f_throughput " "
#         close $f_throughput 

 for {set i 0} {$i < [llength $BER] } {incr i} {
         for {set j 0} {$j < [llength $numflow] } {incr j} {
# puts [lindex $numflow $j] 
# puts [lindex $BER $i]
                for { set k 0 } { $k < [lindex $numflow $j] } { incr k } {
                    exec rm -rf tr.snd.$k 
                    exec rm -rf tr.rcv.$k 
                }
                
                exec ns lpr.voip.tcl $stop $rate $basicrate  [lindex $numflow $j] [lindex $BER $i] >> "auto-result.txt" 2> /dev/null

                # # # # # # # # # 
                set mossum 0.0
                set alldelaysum 0.0
                set alllosssum 0.0
                for { set k 0 } { $k < [lindex $numflow $j] } { incr k } {
                    exec gawk {{print $1}} tr.snd.$k > seq.tr.snd.$k
                    exec gawk {{print $1}} tr.rcv.$k > seq.tr.rcv.$k
# ------------------numbers of snd/rcv    ------------------            
                    set snd_num [exec wc -l seq.tr.snd.$k | awk {{print $1}}]
                    set rcv_num [exec wc -l seq.tr.rcv.$k | awk {{print $1}}]

# ------------------ loss rate ------------------
                    set lossrate [expr ($snd_num.0-$rcv_num.0)/$snd_num.0]
                    set alllosssum [expr $alllosssum+$lossrate]
#                     puts "flow $k loss rate: $lossrate"
# ------------------ delay ------------------
                    set delaysum [exec gawk {{n=n+$4} END {print n}} tr.rcv.$k]
                    set avg_delay [expr $delaysum/$rcv_num+0.125]
                    set alldelaysum [expr $alldelaysum+$avg_delay]
# puts "flow $k delaysum: $delaysum, avg_delay: $avg_delay"
# ------------------ R-factor  -----------------------
                    if { $avg_delay > 177.3 } {
                        set H 1.0
                    } else {
                        set H 0.0
                    }
                    set R_factor [expr 94.2-0.024*$avg_delay-0.11*($avg_delay-177.3)*$H-11.0-40.0*log(1.0+10.0*$lossrate)]
#                     puts "flow $k R-factor: $R_factor"
# ------------------  MoS -----------------------

                    if { $R_factor < 0.0 } {
                        set MoS 1
                    } elseif {  $R_factor > 100.0 } {
                        set MoS 4.5
                    } else  {
                        set MoS [string range [expr 1.0+0.035*$R_factor + 7.0*0.000001*$R_factor*($R_factor-60.0)*(100.0-$R_factor)] 0 4]
                    } 
# puts "flow $k mos: $MoS"
                   set mossum [expr $mossum + $MoS]
                }
                set avg_mos [string range [expr $mossum/[lindex $numflow $j]] 0 4]
                set avg_all_delay [string range [expr $alldelaysum/[lindex $numflow $j]] 0 4]
                set avg_loss_rate [string range [expr $alllosssum/[lindex $numflow $j]] 0 4]

                puts "avg mos: $avg_mos, avg_delay: $avg_all_delay, avg_loss: $avg_loss_rate"
                puts $f_throughput "avg mos: $avg_mos, avg_delay: $avg_all_delay, avg_loss: $avg_loss_rate"
        }
 }
 close $f_throughput
