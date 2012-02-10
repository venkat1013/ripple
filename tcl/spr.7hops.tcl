# 
# 	Tianji Li
# 	Hamilton Institute, NUIM
# 	26/11/2008
# 



# ======================================================================
# Define options
# ======================================================================
# ======================================================================
set opt(chan)           Channel/WirelessChannel    ;# channel type
set opt(prop)           Propagation/Shadowing      ;# radio-propagation model
set opt(netif)          Phy/WirelessPhy            ;# network interface type
set opt(mac)            Mac/802_11                 ;# MAC type
set opt(ifq)            Queue/DropTail/PriQueue    ;# interface queue type
set opt(ll)             LL                         ;# link layer type
set opt(ant)            Antenna/OmniAntenna        ;# antenna model
set opt(ifqlen)         50                         ;# max packet in ifq
set opt(adhocRouting)   NOAH                  ;# routing protocol
set opt(x)              1000                       ;# x coordinate of topology
set opt(y)              1000                       ;# y coordinate of topology
set opt(ShortRetryLimit) 11 
set opt(LongRetryLimit)  11
set opt(RTSThreshold)    65536
set opt(CWmin)           15
set opt(CWmax)           1023
set opt(SlotTime)        0.000009
set opt(SIFS)            0.000016
set opt(AIFS)            2
set opt(PktSize)         1000

set opt(rate)		 [lindex $argv 1]
set opt(dataRate)        $opt(rate)Mb
set opt(basicRate)       [lindex $argv 2]Mb
set opt(PLCPhdrRate)     $opt(rate)Mb

set opt(stop)           [lindex $argv 0]
set opt(start)          0
set opt(rectime)        0.01


set opt(nn)             11
set opt(fnum)           [lindex $argv 3]
set opt(BER)            [lindex $argv 4]
set opt(hops)           [lindex $argv 5]

Mac/802_11 set BER_     $opt(BER)



#------------------------------------------------------------------------
# # use current time as seed for every run
$defaultRNG seed 0

# LL set delay_                   0us ;# by default 25us

#   Application 
#------------------------------------------------------------------------
Agent/TCP               set packetSize_   $opt(PktSize)

# PHY
#------------------------------------------------------------------------
Phy/WirelessPhy set CPThresh_   10000.0         
# Phy/WirelessPhy set CSThresh_   3.36463e-12 ;# no HTP
Phy/WirelessPhy set CSThresh_   2.44547e-08   ;# have HTP
Phy/WirelessPhy set RXThresh_   2.44547e-08     
Phy/WirelessPhy set Pt_         0.28183815      ;# watts

# MAC
#------------------------------------------------------------------------
Mac/802_11 set basicRate_         $opt(basicRate)
Mac/802_11 set dataRate_          $opt(dataRate)
Mac/802_11 set CWMin_             $opt(CWmin)
Mac/802_11 set CWMax_             $opt(CWmax)
Mac/802_11 set SlotTime_          $opt(SlotTime)
Mac/802_11 set SIFS_              $opt(SIFS)
Mac/802_11 set ShortRetryLimit_   $opt(ShortRetryLimit)
Mac/802_11 set LongRetryLimit_    $opt(LongRetryLimit)
Mac/802_11 set RTSThreshold_      $opt(RTSThreshold)
Mac/802_11 set MAX_Sq_SIZE_       $opt(ifqlen) 
Mac/802_11 set num_sta_           $opt(nn)
LL set totNumSTA_                 [expr $opt(nn)-1]

# create simulator instance
#------------------------------------------------------------------------

remove-all-packet-headers
add-packet-header TCP IP LL Mac

set ns_   [new Simulator]
# $ns_ at $opt(rectime) "print_stuff"
set tracefd  [open aaaa.tr w]
set namtrace [open aaaa.nam w]
$ns_ trace-all $tracefd
$ns_ namtrace-all-wireless $namtrace $opt(x) $opt(y)
set topo   [new Topography]
$topo load_flatgrid $opt(x) $opt(y)


create-god        $opt(nn)

# configure for base-station node
$ns_ node-config -adhocRouting $opt(adhocRouting) \
                 -llType $opt(ll) \
                 -macType $opt(mac) \
                 -ifqType $opt(ifq) \
                 -ifqLen $opt(ifqlen) \
                 -antType $opt(ant) \
                 -propType $opt(prop) \
                 -phyType $opt(netif) \
		 -channelType $opt(chan) \
		 -topoInstance $topo \
		 -agentTrace OFF \
                 -ifqTrace OFF \
                 -routerTrace OFF \
                 -macTrace OFF 

#create stations
#------------------------------------------------------------------------
for {set i 0} {$i < $opt(nn)} {incr i} {

    set node_($i) [$ns_ node]   

    if { $i == 0 } {
        $node_($i) set X_ 0.0
        $node_($i) set Y_ 30.0
    }
    if { $i == 1 } {
        $node_($i) set X_ 30.0
        $node_($i) set Y_ 35.0
    }
    if { $i == 2 } {
        $node_($i) set X_ 70.0
        $node_($i) set Y_ 40.0
    }
    if { $i == 3 } {
        $node_($i) set X_ 80.0
        $node_($i) set Y_ 60.0
    }
    if { $i == 4 } {
        $node_($i) set X_ 100.0
        $node_($i) set Y_ 50.0
    }
    if { $i == 5 } {
        $node_($i) set X_ 120.0
        $node_($i) set Y_ 55.0
    }
    if { $i == 6 } {
        $node_($i) set X_ 142.0
        $node_($i) set Y_ 66.0
    }
    if { $i == 7 } {
        $node_($i) set X_ 162.0
        $node_($i) set Y_ 71.0
    }

    if { $i == 8 } {
        $node_($i) set X_ 31.0
        $node_($i) set Y_ 82.0
    }
    if { $i == 9 } {
        $node_($i) set X_ 32.0
        $node_($i) set Y_ 66.0
    }
    if { $i == 10 } {
        $node_($i) set X_ 33.0
        $node_($i) set Y_ 1.0
    }

    $node_($i) set Z_ 0.0
    $node_($i) random-motion 0          ;# disable random motion
}


set flowid 0
for {set i 0} {$i < $opt(fnum)} {incr i 1} {
    set tcp($flowid) [new Agent/TCP]
    set sink($flowid) [new Agent/TCPSink]
    if { $i == 0 } {
        $ns_ attach-agent $node_(0) $tcp($flowid)
        $ns_ attach-agent $node_($opt(hops)) $sink($flowid)
    }
    if { $i == 1 } {
        $ns_ attach-agent $node_(8) $tcp($flowid)
        $ns_ attach-agent $node_(10) $sink($flowid)
    }


    $ns_ connect $tcp($flowid) $sink($flowid)

    $tcp($flowid) set fid_ $i



    if { $opt(hops)==2 } {
      if { $i == 0 } {
          $tcp($flowid)  ForwardList 0 2 -1 -1 -1 -1 -1 -1
          $sink($flowid) ForwardList 2 0 -1 -1 -1 -1 -1 -1
      }
    }
    if { $opt(hops)==3 } {
      if { $i == 0 } {
          $tcp($flowid)  ForwardList 0 3 -1 -1 -1 -1 -1 -1
          $sink($flowid) ForwardList 3 0 -1 -1 -1 -1 -1 -1
      }
    }
    if { $opt(hops)==4 } {
      if { $i == 0 } {
          $tcp($flowid)  ForwardList 0 4 -1 -1 -1 -1 -1 -1
          $sink($flowid) ForwardList 4 0 -1 -1 -1 -1 -1 -1
      }
    }
    if { $opt(hops)==5 } {
      if { $i == 0 } {
          $tcp($flowid)  ForwardList 0 5 -1 -1 -1 -1 -1 -1
          $sink($flowid) ForwardList 5 0 -1 -1 -1 -1 -1 -1
      }
    }
    if { $opt(hops)==6 } {
      if { $i == 0 } {
          $tcp($flowid)  ForwardList 0 6 -1 -1 -1 -1 -1 -1
          $sink($flowid) ForwardList 6 0 -1 -1 -1 -1 -1 -1
      }
    }
    if { $opt(hops)==7 } {
      if { $i == 0 } {
          $tcp($flowid)  ForwardList 0 7 -1 -1 -1 -1 -1 -1
          $sink($flowid) ForwardList 7 0 -1 -1 -1 -1 -1 -1
      }
    }



    if { $i == 1 } {
        $tcp($flowid)  ForwardList 8 9 1 10
        $sink($flowid) ForwardList 10 1 9 8
    }


    $tcp($flowid)  MacType 1
    $sink($flowid) MacType 1


    set app($flowid) [new Application/FTP]
    $app($flowid) attach-agent $tcp($flowid)

    if { $i == 0 } {
        $ns_ at $opt(start) "$app($flowid) start" 
    }
    if { $i == 1 } {
        $ns_ at $opt(start) "$app($flowid) send 5000000" 
    }

    set flowid [expr $flowid + 1]
}







# Result output routines
#########################################################################
$ns_ at $opt(stop) "stop"
$ns_ at $opt(stop) "$ns_ halt"

proc stop {} {
    global ns_ tracefd namtrace node_ opt  tcp sink  
        set now [$ns_ now]

        set flowid 0
        set tempsum 0.0
        set f_throughput  [open "results.7hops.txt" "a"]

#         for {set i 0} {$i < $opt(fnum)} {incr i} {
#                 set t_thru   [string range  [expr [$sink($flowid) set bytes_].0*8/1000000/$now] 0 3]
#                 set tempsum [expr $tempsum+$t_thru]
#                 set flowid [expr $flowid + 1]
#         }

#                 set t_thru   [string range  [expr [$sink(0) set bytes_].0*8/1000000/$now] 0 3]
                set t_thru  [expr [$sink(0) set bytes_].0*8/1000000/$now]
                set tempsum [expr $tempsum+$t_thru]

        puts   "$tempsum"
        puts  -nonewline $f_throughput "[string range $tempsum 0 3] "


        close $f_throughput 

    exec rm -rf aaaa.nam aaaa.tr
}



puts "Starting Simulation..."
$ns_ run

