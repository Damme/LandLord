#!/usr/bin/python3.5

import sys

if len(sys.argv) <2:
  print( "usage:" )
  print("  " + sys.argv[0] + " [.PCK file]", "[Folder-prefix]" )
  print( "  " )
  exit()

import os
from binascii import crc32
from struct import unpack

#get unpack file
src_file_path = sys.argv[1]

#read file length
src_file_size = os.path.getsize(src_file_path)
#read file
b = bytearray(open(src_file_path, 'rb').read())

# crude bytearray to String converter
def toStr(b):
  return "".join(["%c"%i for i in b]).strip("\x00")

# convert 4 bytes to int
def word2int(w):
  return unpack("<I", w)[0]

# crude filetable parser
def getHeader(b, datastart):
  beginstart=b.find(63) #"?"
  filename = toStr(b[0:beginstart]).replace("\\","/")
  beginstart+=1
  beginlength=b.find(63,beginstart) #"?"
  start = int(b[beginstart:beginlength],16) + datastart + 4
  beginlength+=1
  length = int(b[beginlength:beginlength+10],16)
  return {
    "filename": filename,
    "start" : start,
    "length" : length,
    }

#crude way to create nonexisting directories
def checkCreateDir(filename):
  if not os.path.exists(os.path.dirname(filename)):
    try:
        os.makedirs(os.path.dirname(filename))
    except OSError as exc: # Guard against race condition
        if exc.errno != errno.EEXIST:
            raise

def save(filename, data):
  checkCreateDir(prefix + "/" + filename) #create missing directories
  print ("writing %s\t (%d bytes)"%(filename,len(data)))
  open(prefix + "/" + filename, 'wb').write(data)

## Start decoding
print("Unknown byte 0:\t"+ str(b[0]))
print("Unknown byte 1:\t"+ str(b[1]))
print("Unknown byte 2:\t"+ str(b[2]))
print("Unknown byte 3:\t"+ str(b[3]))

prefix = sys.argv[2] if len(sys.argv)>=3 else "."

headerstart_token = bytearray((int(62), int(0x0d), int(0x0a)))
headerstart = b.find(headerstart_token,0)+3
pos=headerstart
datastart_token = bytearray((int(0x0d), int(0x0a), int(0x0d), int(0x0a)))
datastart = b.find(datastart_token,0)
print("header end: \t"+ str(datastart))
nHeaders = b.count(int(13),4,datastart)
print("# files \t: "+str(nHeaders))


# crude way to parse the directory table
files = []
print ("found %i files\n"%nHeaders)

token = bytearray((int(0x0d), int(0x0a)))

for i in range(nHeaders):
  pos2=b.find(token,pos)
  h = getHeader(b[pos:pos2], datastart)
  files.append(h)
  print(h)
  pos = int(pos2)+2

for h in files:
  bFile = b[h["start"]:h["start"]+h["length"]]
#write stripped file
  save(h["filename"],bFile)

print("Done!")
