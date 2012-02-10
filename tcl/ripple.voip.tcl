# 
# 	Tianji Li
# 	Hamilton Institute, NUIM
# 	09/04/2009
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
set opt(adhocRouting)   NOAH                       ;# routing protocol
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

set opt(rate)            [lindex $argv 1]
set opt(dataRate)        $opt(rate)Mb
set opt(basicRate)       [lindex $argv 2]Mb
set opt(PLCPhdrRate)     $opt(rate)Mb

set opt(stop)           [lindex $argv 0]
set opt(start)          0
set opt(rectime)        0.01


set opt(nn)             8
set opt(fnum)           [lindex $argv 3]
set opt(BER)            [lindex $argv 4]
Mac/Ripple set BER_     $opt(BER)


# Codec:  G.711
#------------------------------------------------------------------------
set val(ON)             1500ms
set val(OFF)            1500ms
set val(AppRate)        96Kb
set val(AppSize)        120
#   Application Characteristics 
#------------------------------------------------------------------------
Application/Traffic/Exponential set packetSize_     $val(AppSize) 
Application/Traffic/Exponential set burst_time_     $val(ON)
Application/Traffic/Exponential set idle_time_      $val(OFF)
Application/Traffic/Exponential set rate_           $val(AppRate) 


#------------------------------------------------------------------------
# use current time as seed for every run
$defaultRNG seed 0
# LL set delay_                   0us ;# by default 25us


#   Application 
#------------------------------------------------------------------------
Agent/TCP               set packetSize_   $opt(PktSize)

# PHY
#------------------------------------------------------------------------
Phy/WirelessPhy set CPThresh_   10000.0         
# Phy/WirelessPhy set CSThresh_   3.36463e-12     ;#no HTP
Phy/WirelessPhy set CSThresh_   2.44547e-08 ;# HTP    
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
add-packet-header TCP IP LL  MacRippleData MacRippleAck

set ns_   [new Simulator]
# $ns_ at $opt(rectime) "print_stuff"
set tracefd  [open aaaa.tr w]
set namtrace [open aaaa.nam w]
$ns_ trace-all $tracefd
$ns_ namtrace-all-wireless $namtrace $opt(x) $opt(y)
set topo   [new Topography]
$topo load_flatgrid $opt(x) $opt(y)
create-god [expr $opt(nn)]

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
        $node_($i) set X_ 80.0
        $node_($i) set Y_ 31.0
    }
    if { $i == 5 } {
        $node_($i) set X_ 31.0
        $node_($i) set Y_ 82.0
    }
    if { $i == 6 } {
        $node_($i) set X_ 32.0
        $node_($i) set Y_ 66.0
    }
    if { $i == 7 } {
        $node_($i) set X_ 33.0
        $node_($i) set Y_ 1.0
    }

    $node_($i) set Z_ 0.0
    $node_($i) random-motion 0          ;# disable random motion
}

set flowid 0
for {set i 0} {$i < $opt(fnum)} {incr i 1} {
    set udp($flowid) [new Agent/mUDP]
    set null($flowid) [new Agent/mUdpSink]

    if { $i < 10 } {
        $ns_ attach-agent $node_(0) $udp($flowid)
        $ns_ attach-agent $node_(3) $null($flowid)
    }
    if { $i >= 10 && $i < 20 } {
        $ns_ attach-agent $node_(0) $udp($flowid)
        $ns_ attach-agent $node_(4) $null($flowid)
    }
    if { $i >= 20 && $i < 30} {
        $ns_ attach-agent $node_(5) $udp($flowid)
        $ns_ attach-agent $node_(7) $null($flowid)
    }
    $ns_ connect $udp($flowid) $null($flowid)

    if { $i < 10  } {
        $udp($flowid)  ForwardList 0 1 2 3
    }
    if { $i >= 10 && $i < 20 } {
        $udp($flowid)  ForwardList 0 1 2 4 
    }
    if { $i >= 20 && $i < 30} {
        $udp($flowid)  ForwardList 5 6 1 7 
    }


    $udp($flowid)   set fid_ $i
    $udp($flowid)   MacType 0
    $udp($flowid)   set_filename tr.snd.$i
    $null($flowid)  set_filename tr.rcv.$i

    set cbr($flowid) [new Application/Traffic/Exponential]
    $cbr($flowid) attach-agent $udp($flowid)


    $ns_ at $opt(start) "$cbr($flowid) start" 
    set flowid [expr $flowid + 1]
}




# Result output routines
#########################################################################
$ns_ at $opt(stop) "stop"
$ns_ at $opt(stop) "$ns_ halt"

proc stop {} {
    global ns_ tracefd namtrace node_ opt  tcp sink  
        set now [$ns_ now]

    exec rm -rf aaaa.nam aaaa.tr
}



puts "Starting Simulation..."
$ns_ run

