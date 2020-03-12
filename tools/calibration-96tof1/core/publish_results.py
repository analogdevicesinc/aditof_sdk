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
import paramiko
import json    
import os
import warnings
import logging
import socket

def abort_push(ssh_client, ftp_client, current_dir):
    print('Closing connection\n')
    try:
        # Close connection
        ftp_client.close()
        ssh_client.close()
    except:
        pass
    # Return to starting dir
    os.chdir(current_dir)
    # Re-enable warnings
    warnings.filterwarnings("default")

def push_to_host(local_path):
    # Save cwd
    current_dir = os.getcwd()
    
    # Disable warnings
    warnings.filterwarnings("ignore")
    
    logging.getLogger("paramiko").setLevel(logging.WARN) 
    
    # Disable "DEBUG" logging output for Paramiko
    #logger2 = paramiko.util.logging.getLogger()
    #logger2.setLevel(logging.WARN)
    
    # SSH Setup
    ssh_dict = {}
    with open('config/ssh_config.json') as f:
        ssh_dict = json.load(f)
    ssh_client = paramiko.SSHClient()
    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    # CONNECT
    try:
        print('\nConnecting to device: '+ ssh_dict["username"]+'@'+ssh_dict["hostname"], '...')
        ssh_client.connect( hostname = ssh_dict["hostname"],
                            username = ssh_dict["username"],
                            password = ssh_dict["password"],
                            timeout = 10 )
        # SFTP Setup
        ftp_client = ssh_client.open_sftp()
        print('Connection successful\n')
    except socket.error:
        print('ERROR: Unable to connect to host\n')
        return
    
    # CHANGE CWD
    try:
        # Change to target dir
        os.chdir(local_path) 
    except:
        print('ERROR: Unable to find given path\n')
        abort_push(ssh_client, ftp_client, current_dir)
        return
    
    # REMOTE ROOT
    try:
        print('Creating remote directory ...')
        # Ensure remote root exists
        remote_root = ssh_dict["remote_path"] + '/' + os.path.basename(local_path)
        try:
            ftp_client.chdir(remote_root)
        except IOError:
            ftp_client.mkdir(remote_root)
        print('Remote directory created\n')
    except:
        print('ERROR: Unable to create remote directory\n')
        abort_push(ssh_client, ftp_client, current_dir)
        return

    # TRANSFER
    try:
        print('Beginning file transfer ...')
        # Walk through
        for root, dirs, files in os.walk('.', topdown=True):
            # Esnure remote dir exists
            current_path = remote_root + '/' + root
            try:
                ftp_client.chdir(current_path)
            except IOError:
                ftp_client.mkdir(current_path)
            # Transfer all files
            for afile in files:
                local_file = root + '/' + afile
                dest_file = current_path + '/' + afile
                ftp_client.put(local_file, dest_file)
        print('File transfer successful\n')
    except:
        print('ERROR: Unable to transfer files\n')
        abort_push(ssh_client, ftp_client, current_dir)
        return
    
    # FINISH   
    abort_push(ssh_client, ftp_client, current_dir)
