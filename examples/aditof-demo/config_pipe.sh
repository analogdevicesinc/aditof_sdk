#!/bin/bash
set +x

sudo media-ctl -v -d /dev/media1 -l '"msm_csiphy1":1->"msm_csid0":0[1],"msm_csid0":1->"msm_ispif0":0[1],"msm_ispif0":1->"msm_vfe0_rdi0":0[1]'
sudo media-ctl -v -d /dev/media1 -V '"addi903x 1-0064":0[fmt:SBGGR12/640x960 field:none],"msm_csiphy1":0[fmt:SBGGR12/640x960 field:none],"msm_csid0":0[fmt:SBGGR12/640x960 field:none],"msm_ispif0":0[fmt:SBGGR12/640x960 field:none],"msm_vfe0_rdi0":0[fmt:SBGGR12/640x960 field:none]'
