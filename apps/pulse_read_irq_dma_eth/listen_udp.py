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

# UDP packet sizes
fragment_header_size = 8
fragment_data_size = 1016

# UDP packet fragment count
fragment_count = 4

# timestamp max in unit
counter_period = 4000000000

# timestamp precision in us
counter_precision = 1.0 # 1MHz
counter_precision = 0.1 # 10MHz
counter_precision = 0.0125 # 80MHz

# minimum rate to detect in Hz, if period is higher, continguous
# timestamp analysis is reset
min_rate = 10.0

# UDP packet size
fragment_size = fragment_header_size + fragment_data_size
packet_size = fragment_count*fragment_size
header_size = fragment_count*fragment_header_size
data_size = fragment_count*fragment_data_size

# initialize arrays to store UDP fragments
fragment = bytearray(fragment_size)
header = bytearray(header_size)
data = bytearray(data_size)

timestamps = []
new_timestamps = []
last_timestamps = []

concatenat_timestamps = True

durations = []

wait_duration_start = time.time()
first_fragment_time = time.time()

pulses = 0

while True:

  for fragment_id in range(0, fragment_count):

    # Receive and store whole fragment
    fragment, addr = sock.recvfrom(fragment_size)

    # extract header from fragment
    if header_size > 0:
      header_start = fragment_id * fragment_header_size
      header_end = header_start + fragment_header_size
      header[header_start:header_end] = fragment[:fragment_header_size]

    # extract data from fragment
    data_start = fragment_id * fragment_data_size
    data_end = data_start + fragment_data_size
    data[data_start:data_end] = fragment[fragment_header_size:]

    if fragment_id == 0:
      first_fragment_time = time.time()

  count += 1

  # compute timings involved
  transfer_end_time = time.time()
  wait_duration = transfer_end_time - wait_duration_start
  wait_duration_start = transfer_end_time
  transfer_duration = transfer_end_time - first_fragment_time

  # convert data to uint32 array
  new_timestamps = list(struct.unpack('I' * int(len(data) / 4), data))

  # remove trailing magic value if any
  new_timestamps = [x for x in new_timestamps if x < counter_period]

  pulses += len(new_timestamps)

  # print it for information
  print("{}: fragments: {}, waited: {:6.2f}ms, transfer: {:3.0f}us, pulses: {}".format(
    count, fragment_count, wait_duration*1e3, transfer_duration*1e6, pulses))

  # if we waited too long, throw away last timestamps
  if wait_duration > 1/min_rate:
    last_timestamps = []

  # concatenate with last timestamps to ensure continuity
  # if we did not wait too long
  if concatenat_timestamps:
    timestamps = last_timestamps + new_timestamps
    last_timestamps = new_timestamps
  else:
    timestamps = new_timestamps

  # unwrap timestamps according to counter period
  for i in range(1, len(timestamps)):
    while timestamps[i] - timestamps[i-1] < -counter_period/2:
      timestamps[i] += counter_period

  # convert timestamps to float
  for i in range(len(timestamps)):
    timestamps[i] = float(timestamps[i])

  if len(timestamps) == 1:
    continue

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
  print("  average: {:.2f}us, dev: {:5.2f}us, min: {:5.2f}us,"
    " max: {:5.2f}us, rate: {:.2f}kHz".format(
    average, std_dev, min, max, rate*1000))

  if std_dev > 10:
    print(
      "%%%%%%%%%%%%%%%%\n"
      "% std_dev > 10 %\n"
      "%%%%%%%%%%%%%%%%"
      )
    exit(-1)
