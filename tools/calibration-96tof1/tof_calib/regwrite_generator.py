#
# BSD 3-Clause License
#
# Copyright (c) 2019, Analog Devices, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# This class is used to generate delay register writes
import re

class regwrite_generator(object):

    def __init__(self, seq_file):
        self.code_dict = {}
        self.data_dict = {}
        self.seq_file = seq_file
    
    def create_code_dict(self, text):
        reg = re.compile(r'([0-9a-f]{4}  [0-9a-f]{4})')
        rawinfo = re.findall(reg, text)
        for x in rawinfo:
            s_line = re.split(r'\s', x)
            addr = int(s_line[0],16)
            data = int(s_line[2],16)
            self.code_dict[addr] = data
        return self.code_dict
        
    def create_seq_info(self):
        data_name = ['PulseCount', 'LD1_Tap', 'LD2_Tap', 'LD3_Tap', 'LD4_Tap', 'LD5_Tap', 'Pos_Off', 'Vec_Off', 'Start_Loc', 'Tbl_Len']
        reg = re.compile(r'([0-9a-zA-Z]+)')
        myfile = open(self.seq_file, 'r')
        for line in myfile:
            rawInfo = re.findall(reg, line)
            if len(rawInfo) == 1:
                currLabel = rawInfo[0]
            if len(rawInfo) == 4:
                curr_mode = rawInfo[1]
                curr_seq = rawInfo[3]
                i = 0
                if curr_mode in self.data_dict:
                    self.data_dict[curr_mode][curr_seq] = {}
                else:
                    self.data_dict[curr_mode] = {}
                    self.data_dict[curr_mode][curr_seq] = {}
                for i in range(10):
                    rawInfo = re.findall(reg, myfile.readline())
                    self.data_dict[curr_mode][curr_seq][data_name[i]] = [int(rawInfo[0], 16), int(rawInfo[1], 16)] 
        myfile.close()
        return self.data_dict
        
        # Given mode, sweep specified ld for all sequences
    def delay_sequences(self, mode, delay, ld):
        delay_writes = {}

        for x in self.data_dict[str(mode)]:
            writes = self.delay_sequence_ld(delay, ld, self.data_dict[str(mode)][x])
            delay_writes = dict(delay_writes, **writes)
                
        return delay_writes
        
    def generate_delay_writes(self, mode, delay_min, delay_max, ld):
        writes_dict = {}
        for x in range(delay_min, delay_max):
            writes_dict[x] = self.delay_sequences(mode, x, ld)

        return writes_dict
            
    def setbit(self, bit, vec):
        bit = 1 << bit
        vec = vec | bit 
        return vec

    def unsetbit(self, bit, vec):
        bit = 1 << bit
        bit = ~bit
        vec = vec & bit 
        return vec
        
    def get_blanking_values(self, ld, seq_dict):
        
        pos_len = seq_dict['Tbl_Len'][1] & 0x00ff
        vec_len = (seq_dict['Tbl_Len'][1] & 0xff00) >> 8

        if pos_len != vec_len:
            print('Table length not equal')

        start_loc = seq_dict['Start_Loc'][1]
        pos_len = seq_dict['Tbl_Len'][1] & 0x00ff
        vec_len = (seq_dict['Tbl_Len'][1] & 0xff00) >> 8
        pos_ptr = (seq_dict['Pos_Off'][1] * 2) + 0x4000
        vec_ptr = (seq_dict['Vec_Off'][1] * 2) + 0x4000

        blk_pos = -1
        blk_neg = -1

        for i in range(vec_len):
            curr_vec = self.code_dict[vec_ptr + i]
            if ((curr_vec >> (ld - 1)) & 0x0001) == 1:
                if blk_pos == -1:
                    blk_pos = i
                elif blk_neg == -1:
                    blk_neg = i

        start_pos = start_loc + 2
        pos_tbl = []
        for i in range(pos_len):
            if i == 0:
                pos_tbl.append(self.code_dict[pos_ptr+i] + start_pos)
            else:
                pos_tbl.append(self.code_dict[pos_ptr+i] + pos_tbl[i-1])

        blk_pos = pos_tbl[blk_pos]
        blk_neg = pos_tbl[blk_neg]            

        return blk_pos, blk_neg
        
    # Delay Sequence LD
    def delay_sequence_ld(self, delay, ld, seq_dict):

        taps = seq_dict['LD' + str(ld) + '_Tap'][1]
        taps_addr = seq_dict['LD' + str(ld) + '_Tap'][0]
        
        tap_pos = taps & 0x00ff
        tap_neg = (taps & 0xff00) >> 8

        blk_pos, blk_neg = self.get_blanking_values(ld, seq_dict)

        blk_pos_shift = 0
        blk_neg_shift = 0

        tap_pos = tap_pos + delay
        tap_neg = tap_neg + delay

        while tap_pos >= 128:
            blk_pos_shift += 1
            tap_pos -= 128

        while tap_neg >= 128:
            blk_neg_shift += 1
            tap_neg -= 128

        while tap_pos < 0:
            blk_pos_shift -= 1
            tap_pos += 128

        while tap_neg < 0:
            blk_neg_shift -= 1
            tap_neg += 128

        blk_pos = blk_pos + blk_pos_shift
        blk_neg = blk_neg + blk_neg_shift
        
        tap_write = {}
        tap_write[hex(taps_addr)] = (tap_neg << 8) + tap_pos
        blk_writes = self.set_blanking_values(blk_pos, blk_neg, ld, seq_dict)

        writes = dict(tap_write, **blk_writes)
        
        return writes
        
    # Set blanking vals
    def set_blanking_values(self, blk_pos, blk_neg, ld, seq_dict):
        start_loc = seq_dict['Start_Loc'][1]
        pos_len = seq_dict['Tbl_Len'][1] & 0x00ff
        vec_len = (seq_dict['Tbl_Len'][1] & 0xff00) >> 8
        pos_ptr = (seq_dict['Pos_Off'][1] * 2) + 0x4000
        vec_ptr = (seq_dict['Vec_Off'][1] * 2) + 0x4000

        start_pos = start_loc + 2
        pos_tbl = []
        for i in range(pos_len):
            if i == 0:
                pos_tbl.append(self.code_dict[pos_ptr+i] + start_pos)
            else:
                pos_tbl.append(self.code_dict[pos_ptr+i] + pos_tbl[i-1])

        blk_pos_loc = pos_tbl.index(blk_pos)
        blk_neg_loc = pos_tbl.index(blk_neg)

        blk_writes = {}
        for i in range(vec_len):
            if i == blk_pos_loc:
                curr_vec = self.setbit(ld-1, self.code_dict[vec_ptr + i])
            elif i == blk_neg_loc:
                curr_vec = self.setbit(ld-1, self.code_dict[vec_ptr + i])
            else:
                curr_vec = self.unsetbit(ld-1, self.code_dict[vec_ptr + i])
            blk_writes[hex(vec_ptr + i)] = curr_vec

        return blk_writes
        

        
