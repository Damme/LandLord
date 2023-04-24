import os
import binascii

# Define the input and output file paths
input_file = "main.bin"
output_file = "db504_open.bin"

# Check if the input file exists
if not os.path.exists(input_file):
    print("Input file not found!")
    exit()

# Read the contents of the input file
with open(input_file, "rb") as f:
    data = f.read()

# Calculate the CRC32 checksum
crc = binascii.crc32(data)

# Convert the CRC32 checksum to a byte string and append it to the beginning of the data
crc = crc.to_bytes(4, byteorder="little")
new_data = crc + data

# Write the new data to the output file
with open(output_file, "wb") as f:
    f.write(new_data)
