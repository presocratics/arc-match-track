#!/usr/bin/env sh
# preprocess.sh
# Martin Miller
# Created: 2014/08/20
# Run this on a directory of raw data to generate the files needed for
# reflection tracking.
# Usage: preprocess.sh INDIR OUTDIR NAME
SLAMDIR=~/ARC/slam
indir=$1
outdir=$2
name=$3
if [ $# -ne 3 ]
then
    echo "Usage: preprocess.sh indir outdir name"
    exit
fi
$SLAMDIR/bin/downsample $indir/framedata $indir/attitude | \
    cut -d, -f2,3,4 | \
    $SLAMDIR/bin/euler2qbw > $outdir/${name}.img &
< $indir/framedata cut -d, -f2| sed "s:\([0-9]*\):$indir/img/\1.jpg:" > \
    $outdir/${name}.qbw




