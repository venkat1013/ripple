# 
#       Tianji Li
#       Hamilton Institute, NUIM
#       09/04/2009
#       Modified from Ravi Prasad's code at http://www.cc.gatech.edu/~ravi/buffer.htm
#       Web-like flows: Create successive short transfers within single TCP conn
#


set opt(sleeptm)        0.5  ;# exponential sleep time
set opt(avg_size)       80   ;# average flow size, pareto average???   
set opt(pareto_shape)   1.5  ;# ???



set rnd [new RNG]
$rnd seed "date +%s"

proc actv_mouse {tcp ftp sleeptm avg_size pareto_shape} {
global ns_ rnd  opt

    set rngpareto [new RandomVariable/Pareto];
    $rngpareto   set avg_   $avg_size;
    $rngpareto   set shape_ $pareto_shape;

    set maxpkts [$rngpareto value]
    set maxpkts [expr $maxpkts + 0.9 ] ;
    set maxpkts [expr int ($maxpkts)] ;

# exponential sleep time
    set prd_time [expr [$rnd exponential] * $sleeptm + [$ns_ now]]

# pareto connection size
    $ns_ at $prd_time "$ftp producemore $maxpkts"
    $ns_ at $prd_time "actv_mouse $tcp $ftp $opt(sleeptm) $opt(avg_size) $opt(pareto_shape)" 
}

