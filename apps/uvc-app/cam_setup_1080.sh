#!/bin/bash
set +x

#Reset all
sudo media-ctl -d /dev/media1 -l '"msm_csiphy0":1->"msm_csid0":0[0],"msm_csid0":1->"msm_ispif0":0[0],"msm_ispif0":1->"msm_vfe0_rdi0":0[0],"msm_ispif0":1->"msm_vfe0_pix":0[0]'
sudo media-ctl -d /dev/media1 -l '"msm_csiphy1":1->"msm_csid1":0[0],"msm_csid1":1->"msm_ispif1":0[0],"msm_ispif1":1->"msm_vfe0_rdi1":0[0],"msm_ispif1":1->"msm_vfe0_pix":0[0]'

# Connect CSI0 to ISP0 output
sudo media-ctl -d /dev/media1 -l '"msm_csiphy0":1->"msm_csid0":0[1],"msm_csid0":1->"msm_ispif0":0[1],"msm_ispif0":1->"msm_vfe0_rdi0":0[1]'
# Set resolution to 1920x1080
sudo media-ctl -d /dev/media1 -V '"ov5640 1-0078":0[fmt:UYVY8_2X8/1920x1080 field:none],"msm_csiphy0":0[fmt:UYVY8_2X8/1920x1080 field:none],"msm_csid0":0[fmt:UYVY8_2X8/1920x1080 field:none],"msm_ispif0":0[fmt:UYVY8_2X8/1920x1080 field:none],"msm_vfe0_rdi0":0[fmt:UYVY8_2X8/1920x1080 field:none]'
