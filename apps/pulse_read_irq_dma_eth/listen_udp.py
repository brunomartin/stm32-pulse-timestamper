# This script listen uint32_t timestamps array on UDP port
# To avoid UDP packet mixing, it must be executed on the network
# "near" the pulse timestamper, e.g., not on wifi

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

# timestamp max in unit
counter_period = 4000000000

# timestamp precision in us
counter_precision = 0.1

# UDP packet size
packet_size = fragment_count*fragment_size

# initialize arrays to store UDP fragments
data = bytearray(packet_size)

timestamps = []
for i in range(int(packet_size/4)):
  timestamps.append(0.)

durations = []
for i in range(int(packet_size/4)):
  durations.append(0.)

wait_duration_start = time.time()
first_fragment_time = time.time()

while True:

  start = time.time()

  for fragment in range(0, fragment_count):

    data_start = fragment * fragment_size
    data_end = data_start + fragment_size

    data[data_start:data_end], addr = sock.recvfrom(fragment_size)

    if fragment == 0:
      first_fragment_time = time.time()

  count += 1

  # compute timings involved
  transfer_end_time = time.time()
  wait_duration = transfer_end_time - wait_duration_start
  wait_duration_start = transfer_end_time
  transfer_duration = transfer_end_time - first_fragment_time

  # print it for information
  print("{}: fragments: {}, waited: {:.2f}ms, transfer:{:.2f}ms".format(
    count, fragment_count, wait_duration*1000, transfer_duration*1000))

  # convert data to uint32 array
  timestamps = list(struct.unpack('I' * int(len(data) / 4), data))

  # Check if any bad value
  for i in range(len(timestamps)):
    if timestamps[i] >= counter_period:
      print("!!!! timestamps[{}]: 0xFFFFFFFF".format(i))

  # unwrap timestamps according to counter period
  for i in range(1, len(timestamps)):
    while timestamps[i] - timestamps[i-1] < -counter_period/2:
      timestamps[i] += counter_period

  # UDP packet may not arrived in order
  # Nervertheless, they came from the same line
  # so we can sort them
  timestamps.sort()

  # convert timestamps to float
  for i in range(len(timestamps)):
    timestamps[i] = float(timestamps[i])

  # compute durations from timestamps
  durations = [(timestamps[i] - timestamps[i-1]) for i in range(1, len(timestamps)-1)]

  # apply conversion according to precision
  durations = [ duration*counter_precision for duration in durations]

  # compute statistics
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

  rate = math.nan
  if average != 0:
    rate = 1/average

  # print statistics
  print("  average:{:.2f}us, dev:{:.2f}us, min: {:.2f}us, max: {:.2f}us, rate: {:.2f}kHz".format(
    average, std_dev, min, max, rate*1000))

  if std_dev > 20:
    for i in range(len(timestamps)):
      duration = 0
      if i > 0:
        duration = timestamps[i] - timestamps[i-1]

      print("{}: {}, {}".format(i, timestamps[i], duration))
