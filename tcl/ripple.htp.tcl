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
set opt(mac)            Mac/Ripple                 ;# MAC type
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


set opt(nn)             [expr [lindex $argv 3]*4]
set opt(fnum)           [lindex $argv 3]
set opt(BER)            [lindex $argv 4]
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
Mac/Ripple set basicRate_         $opt(basicRate)
Mac/Ripple set dataRate_          $opt(dataRate)
Mac/Ripple set CWMin_             $opt(CWmin)
Mac/Ripple set CWMax_             $opt(CWmax)
Mac/Ripple set SlotTime_          $opt(SlotTime)
Mac/Ripple set SIFS_              $opt(SIFS)
Mac/Ripple set ShortRetryLimit_   $opt(ShortRetryLimit)
Mac/Ripple set LongRetryLimit_    $opt(LongRetryLimit)
Mac/Ripple set RTSThreshold_      $opt(RTSThreshold)
Mac/Ripple set MAX_Sq_SIZE_       $opt(ifqlen) 
Mac/Ripple set num_sta_           $opt(nn)
LL set totNumSTA_                 [expr $opt(nn)-1]

# create simulator instance
#------------------------------------------------------------------------

remove-all-packet-headers
add-packet-header TCP IP LL MacRippleData MacRippleAck

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

# 
    if { $i == 0 } {
        $node_($i) set X_ [expr $i+0.0]
        $node_($i) set Y_ [expr $i+30.0]
    }
    if { $i == 1 } {
        $node_($i) set X_ [expr $i+30.0]
        $node_($i) set Y_ [expr $i+35.0]
    }
    if { $i == 2 } {
        $node_($i) set X_ [expr $i+70.0]
        $node_($i) set Y_ [expr $i+40.0]
    }
    if { $i == 3 } {
        $node_($i) set X_ [expr $i+80.0]
        $node_($i) set Y_ [expr $i+60.0]
    }
# hidden nodes
    if { [expr $i % 4] == 0 && $i > 0 } {
        $node_($i) set X_ [expr $i+120.0]
        $node_($i) set Y_ [expr $i+31.0]
    }
    if { [expr $i % 4] == 1 && $i > 1  } {
        $node_($i) set X_ [expr $i+150.0]
        $node_($i) set Y_ [expr $i+32.0]
    }
    if { [expr $i % 4] == 2 && $i > 2} {
        $node_($i) set X_ [expr $i+180.0]
        $node_($i) set Y_ [expr $i+33.0]
    }
    if { [expr $i % 4] == 3 && $i > 3 } {
        $node_($i) set X_ [expr $i+210.0]
        $node_($i) set Y_ [expr $i+34.0]
    }

    $node_($i) set Z_ 0.0
    $node_($i) random-motion 0          ;# disable random motion
}


set flowid 0
for {set i 0} {$i < $opt(fnum)} {incr i 1} {
    set tcp($flowid) [new Agent/TCP]
    set sink($flowid) [new Agent/TCPSink]
    $ns_ attach-agent $node_([expr $i * 4 ]) $tcp($flowid)
    $ns_ attach-agent $node_([expr $i * 4 + 3]) $sink($flowid)

    $ns_ connect $tcp($flowid) $sink($flowid)

    $tcp($flowid) set fid_ $i


    $tcp($flowid)  ForwardList [expr $i*4 ] [expr $i*4+1 ] [expr $i*4+2 ] [expr $i*4+3 ]
    $sink($flowid) ForwardList [expr $i*4+3 ] [expr $i*4+2 ] [expr $i*4+1 ] [expr $i*4 ]

    $tcp($flowid)  MacType 0 
    $sink($flowid) MacType 0


    set app($flowid) [new Application/FTP]
    $app($flowid) attach-agent $tcp($flowid)

    if { $i == 0 } {
        $ns_ at $opt(start) "$app($flowid) start" 
    } else {
        $ns_ at $opt(start) "$app($flowid) send 5000000" 
    }
    set flowid [expr $flowid + 1]
}







# Result output routines
#########################################################################
$ns_ at $opt(stop) "stop"
$ns_ at $opt(stop) "$ns_ halt"

proc stop {} {
    global ns_ tracefd namtrace node_ opt  tcp sink  argv
        set now [$ns_ now]

        set flowid 0
        set tempsum 0.0
        set f_throughput  [open "results.htp.txt" "a"]
        puts $f_throughput "================================================="
        puts $f_throughput "RIPPLE [lindex $argv 3] flows"
        puts $f_throughput "================================================="
        set aaa "+DATE(month/day/year): %m/%d/%Y, TIME: %H:%M:%S"
        set systemtime       "[exec date $aaa]"
        puts $f_throughput "$systemtime"
        puts $f_throughput "rate: $opt(rate) Mbps"
        puts $f_throughput "run: $opt(stop) seconds"
        puts $f_throughput "BER= $opt(BER)"
        puts $f_throughput "================================================="
        puts -nonewline  $f_throughput "flows: "
       
        for {set i 0} {$i < $opt(fnum)} {incr i} {
                set t_thru   [string range  [expr [$sink($flowid) set bytes_].0*8/1000000/$now] 0 3]
                set tempsum [expr $tempsum+$t_thru]
                if { $i == 0 } {
                    set thru1 $tempsum
                }
                set flowid [expr $flowid + 1]
        }
        puts  -nonewline $f_throughput "  sum: [string range $tempsum 0 3]="
        set flowid 0
        for {set i 0} {$i < $opt(fnum)} {incr i} {
                set t_thru   [string range  [expr [$sink($flowid) set bytes_].0*8/1000000/$now] 0 3]
                puts -nonewline  $f_throughput  "[string range $t_thru 0 3]"
                if { $i < $opt(fnum) - 1 } {
                  puts -nonewline  $f_throughput  "/"
                }
                set flowid [expr $flowid + 1]
        }
        puts $f_throughput  ""


        close $f_throughput 


        set f_10  [open "throughput.htp.txt" "a"]
#         puts -nonewline  $f_10 "[string range $tempsum 0 3]  "
        puts -nonewline  $f_10 "[string range $thru1 0 3]  "
        close $f_10


        exec rm -rf aaaa.nam aaaa.tr
}



puts "Starting Simulation..."
$ns_ run

