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
fragment_count = 4

# UDP packet size
packet_size = fragment_count*fragment_size

# byte array in which to store UDP fragments
data = bytearray(packet_size)
part = bytearray(fragment_size)
first = -1
last = -1

wait_duration_start = time.time()
first_fragment_time = time.time()

while True:

  start = time.time()

  for fragment in range(0, fragment_count):

    data_start = fragment * fragment_size
    data_end = data_start + fragment_size

    part, addr = sock.recvfrom(fragment_size)
    data[data_start:data_end] = part

    if fragment == 0:
      first_fragment_time = time.time()

  count += 1

  transfer_end_time = time.time()

  wait_duration = transfer_end_time - wait_duration_start
  wait_duration_start = time.time()

  transfer_duration = transfer_end_time - first_fragment_time

  first = struct.unpack('I', data[:4])
  last = struct.unpack('I', data[-4:])
  
  print("{}: fragments: {}, waited: {:.2f}ms, transfer:{:.2f}ms".format(
    count, fragment_count, wait_duration*1000, transfer_duration*1000))

  print("  first: {}, last: {}".format(first, last))
  