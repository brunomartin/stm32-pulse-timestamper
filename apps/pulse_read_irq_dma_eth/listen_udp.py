import socket
import struct
import time
import math

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

  # convert data to uint32 array
  timestamps = list(struct.unpack('I' * int(len(data) / 4), data))
  durations = [(timestamps[x+1] - timestamps[x]) for x in range(len(timestamps)-1)]

  average = 0
  min = durations[0]
  max = durations[0]
  for duration in durations:
    average += duration
    min = duration if duration < min else min
    max = duration if duration > max else max
  average /= len(durations)

  std_dev = 0
  for duration in durations:
    std_dev += (duration - average)**2
  std_dev /= len(durations)
  std_dev = math.sqrt(std_dev)

  print("{}: fragments: {}, waited: {:.2f}ms, transfer:{:.2f}ms".format(
    count, fragment_count, wait_duration*1000, transfer_duration*1000))

  print("  average:{:.2f}us, dev:{:.2f}us, min: {}us, max: {}us".format(average, std_dev, min, max))
