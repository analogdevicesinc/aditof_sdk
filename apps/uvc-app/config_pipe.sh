#!/bin/bash
set +x

sudo rmmod g_webcam

sudo media-ctl -v -d /dev/media0 -l '"msm_csiphy1":1->"msm_csid0":0[1],"msm_csid0":1->"msm_ispif0":0[1],"msm_ispif0":1->"msm_vfe0_rdi0":0[1]'
sudo media-ctl -v -d /dev/media0 -V '"addi903x 4-0064":0[fmt:SBGGR12/640x960 field:none],"msm_csiphy1":0[fmt:SBGGR12/640x960 field:none],"msm_csid0":0[fmt:SBGGR12/640x960 field:none],"msm_ispif0":0[fmt:SBGGR12/640x960 field:none],"msm_vfe0_rdi0":0[fmt:SBGGR12/640x960 field:none]'

sudo modprobe g_webcam streaming_maxpacket=3072

BASEDIR=$(dirname $0)

sudo ./$BASEDIR/uvc-gadget -r 2 -o 1 -a -n 4
