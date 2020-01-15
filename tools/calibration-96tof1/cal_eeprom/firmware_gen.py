import click
import struct

def extract_code_block(lf_filename):
   f = open(lf_filename)
   address = []
   data = []
   mode_locations={}
   lineNum=0
   for line in f.readlines():
       # Clean up line to remove comments and extract hexa codes
       removedChars = ' \t\n\r'
       for char in removedChars:
         line = line.replace(char,"")

       if (len(line)>=8):
         try:
           addr=int(line[0:4],16)
           dat=int(line[4:8],16)
           address.append(addr)
           data.append(dat)
           # Check if there is a //.. or /* ... */ describing pattern, mode, field and sequence numbers
           if '//' in line:
             name=line.split('//',1)[1]
             mode_locations[name] = lineNum
           if '/*' in line:
             name=line[line.index('/*')+2:line.index('*/')]
             mode_locations[name] = lineNum
           lineNum+=1

         except ValueError:
           continue
   f.close()
   return address, data, mode_locations


def generate_bin(addr_data_tuple, bin_file_name):
    with open(bin_file_name, 'wb') as f:
        for addr,data in addr_data_tuple:
           # convert to binary data - little endian
           bin_addr = struct.pack('>i', addr)
           bin_data = struct.pack('>i', data)
           f.write(bin_addr)
           f.write(bin_data)


@click.command()
@click.argument('lf_filename', type=click.STRING)
@click.argument('bin_filename', type=click.STRING)
def gen_firmware(lf_filename, bin_filename):
    addr_data_tuple, mode_locations = extract_code_block(lf_filename)
    generate_bin(addr_data_tuple, bin_filename)


if __name__ == "__main__":
    gen_firmware()
