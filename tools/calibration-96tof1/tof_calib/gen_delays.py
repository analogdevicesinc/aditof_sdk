# Script for generating delay register writes

from natsort import natsorted, ns
import os
import re
import json
from . import regwrite_generator as rg


def generate_delays(hpt_data_delayfile, data_delayfile, min_delay, max_delay, seq_file ):
     rg1 = rg.regwrite_generator(seq_file)

     with open(hpt_data_delayfile, 'r') as myfile:
          data=myfile.read().replace('\n', '')
     timing_code = rg1.create_code_dict(data)
     myfile.close()
     with open(data_delayfile, 'r') as myfile:
         data=myfile.read().replace('\n', '')
     timing_code = rg1.create_code_dict(data)
     myfile.close()

     rg1.create_seq_info()

     
     # Mode to delay
     mode = 0
     
     # Which LD1-7 is being swept
     ld = 5

     delay_dict = rg1.generate_delay_writes(mode, min_delay, max_delay, ld)

     file = open('delays.json', 'w')
     json.dump(delay_dict, file)
     return delay_dict
