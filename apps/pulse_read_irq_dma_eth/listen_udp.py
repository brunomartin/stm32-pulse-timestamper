import socket
import struct
import time

UDP_IP = ""
UDP_PORT = 8042

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

count = 0

# UDP packet fragment size
fragment_size = 1024

# UDP packet fragment count
fragment_count = 16

# UDP packet size
packet_size = fragment_count*fragment_size

# byte array in which to store UDP fragments
data = bytearray(packet_size)
part = bytearray(fragment_size)
first = -1
last = -1

while True:
  start = time.time()
  for fragment in range(0, fragment_count):

    # _, addr = sock.recvfrom(fragment_size)

    data_start = fragment * fragment_size
    data_end = data_start + fragment_size
    part, addr = sock.recvfrom(fragment_size)
    data[data_start:data_end] = part

  end = time.time()

  count += 1

  first = struct.unpack('I', data[:4])
  last = struct.unpack('I', data[-4:])
  
  print("{}: fragments: {}, elapsed: {:.2f}ms, first: {}, last: {}".format(
    count, fragment_count, (end - start)*1000, first, last))