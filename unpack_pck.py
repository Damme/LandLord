#!/usr/bin/python3

import sys
import os
import binascii
from struct import unpack

# Convert bytearray to string
def bytearray_to_str(data):
    return "".join(["%c" % i for i in data]).strip("\x00")

# Convert 4 bytes to a 32-bit word
def bytes_to_word(byte_data):
    return unpack("<I", byte_data)[0]

# Parse file header
def parse_header(data, data_start):
    # Split the data using "?" as the delimiter
    parts = data.split(b'?')

    # Get the filename, start, and length values
    filename = bytearray_to_str(parts[0]).replace("\\", "/")
    start = int(parts[1], 16) + data_start + 4

    # If there are 4 parts, ignore the middle part
    special = 0
    if len(parts) == 4:
        length = int(parts[3], 16)
        special = int(parts[2], 16)
    else:
        length = int(parts[2], 16)

    return {
        "filename": filename,
        "start": start,
        "length": length,
        "special": special,
    }


# Ensure directory exists
def ensure_dir_exists(filename):
    directory = os.path.dirname(filename)
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                raise

# Save file
def save_file(filename, prefix, data):
    ensure_dir_exists(prefix + "/" + filename)
    print(f"writing {filename}\t ({len(data)} bytes)")
    open(prefix + "/" + filename, 'wb').write(data)

def main():
    # Check command line arguments
    if len(sys.argv) < 2:
        print("Usage:")
        print(f"  {sys.argv[0]} [.PCK file] [Folder-prefix]")
        print()
        exit()

    # File to unpack
    src_file_path = sys.argv[1]
    # File size
    src_file_size = os.path.getsize(src_file_path)
    # Read file into bytearray
    binary_data = bytearray(open(src_file_path, 'rb').read())
    
    crc = binascii.crc32(binary_data[4:], 0x0)
    print(f"Calculated checksum: {hex(crc)}")
    
    # Start decoding
    #print(f"Unknown byte 0:\t{binary_data[0]}")
    #print(f"Unknown byte 1:\t{binary_data[1]}")
    #print(f"Unknown byte 2:\t{binary_data[2]}")
    #print(f"Unknown byte 3:\t{binary_data[3]}")
    print(f"Unknown first 32-bit value, on db504 firmware this is checksum of binary from byte 4 and forward: {hex(bytes_to_word(binary_data[0:4]))}")
    
    if binary_data[4] != int(0x3c):
        #print(f"Unknown byte 4:\t{binary_data[4]}")
        #print(f"Unknown byte 5:\t{binary_data[5]}")
        #print(f"Unknown byte 6:\t{binary_data[6]}")
        #print(f"Unknown byte 7:\t{binary_data[7]}")
        print(f"Unknown second 32-bit value: {hex(bytes_to_word(binary_data[4:8]))}")
    
    print(f"Unknown last 32-bit value, on db504 firmware this is always 0x1d38d491 : {hex(bytes_to_word(binary_data[-4:]))}")

    prefix = sys.argv[2] if len(sys.argv) >= 3 else "."

    header_start_token = bytearray((int(0x3e), int(0x0d), int(0x0a)))
    header_start = binary_data.find(header_start_token, 0) + 3
    pos = header_start
    data_start_token = bytearray((int(0x0d), int(0x0a), int(0x0d), int(0x0a)))
    data_start = binary_data.find(data_start_token, 0)
    print(f"header end: \t{data_start}")
    num_files = binary_data.count(int(13), 4, data_start)
    print(f"# files \t: {num_files}")

    # Parse directory table
    files = []
    print(f"found {num_files} files\n")

    token = bytearray((int(0x0d), int(0x0a)))

    for _ in range(num_files):
        pos2 = binary_data.find(token, pos)
        header = parse_header(binary_data[pos:pos2], data_start)
        files.append(header)
        print(header)
        pos = int(pos2) + 2

    for header in files:
        file_data = binary_data[header["start"]:header["start"] + header["length"]]
        save_file(header["filename"], prefix, file_data)
    
    print("Done!")

if __name__ == '__main__':
    main()
