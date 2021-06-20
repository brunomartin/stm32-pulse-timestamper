# This script listen uint32_t timestamps array on UDP port
# To avoid UDP packet mixing, it must be executed on the network
# "near" the pulse timestamper, e.g., not on wifi

import socket
import struct
import time
import math

UDP_IP = ""
UDP_PORT = 8042

lines = 2

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

# UDP packet sizes
fragment_header_size = 12
fragment_data_size = 1000 # 250x4

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

# Tell if concatenate timestamp to be sure packet are contiguous
concatenate_packets = True

# Tell if we want to compute statistics or not
compute_stats = False

# UDP packet size
fragment_size = fragment_header_size + fragment_data_size
packet_size = fragment_count*fragment_size
header_size = fragment_count*fragment_header_size
data_size = fragment_count*fragment_data_size

# initialize arrays to store UDP fragments
fragment = bytearray(fragment_size)
header = bytearray(header_size)
data = bytearray(data_size)

# initialize lists for each line
timestamps = []
last_headers = []
last_timestamps = []

last_packet_id = []

wait_duration_start = []
first_fragment_time = []

pulses = []

for i in range(lines):
  last_headers.append([])
  last_timestamps.append([])
  last_packet_id.append(-1)
  wait_duration_start.append(time.time())
  pulses.append(0)

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

  # unpack header content : line, packet_id, fragment_id
  new_headers = list(struct.unpack('III' * fragment_count, header))

  line = new_headers[0]
  packet_id = new_headers[1]
  fragment_id = new_headers[2]

  # compute timings involved
  transfer_end_time = time.time()
  wait_duration = transfer_end_time - wait_duration_start[line]
  wait_duration_start[line] = transfer_end_time
  transfer_duration = transfer_end_time - first_fragment_time

  # if we waited too long, throw away last timestamps
  if wait_duration > 1/min_rate:
    last_headers[line] = []
    last_timestamps[line] = []
    last_packet_id[line] = -1
    wait_duration = 0

  process_time = time.time()

  # convert data to uint32 array
  new_timestamps = list(struct.unpack('{}I'.format(int(len(data) / 4)), data))

  # Detect discontinuation in packet ids
  if last_packet_id[line] == -1:
    last_packet_id[line] = packet_id
  else:
    if packet_id != last_packet_id[line] + 1:
      print(fragment)
      print(new_headers)
      print(new_timestamps)
      print(
        "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
        "% packet_id ({}) != last_packet_id + 1 ({}) %\n"
        "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
        .format(packet_id, last_packet_id + 1))
      exit(-1)

    last_packet_id[line] = packet_id

  # remove trailing magic value if any
  new_timestamps = [x for x in new_timestamps if x < counter_period]

  # Increment total number of pulses from script start
  pulses[line] += len(new_timestamps)

  if compute_stats:

    # concatenate with last timestamps to ensure continuity
    # if we did not wait too long
    if concatenate_packets:
      headers = last_headers[line] + new_headers
      last_headers[line] = new_headers

      timestamps = last_timestamps[line] + new_timestamps
      last_timestamps[line] = new_timestamps
    else:
      headers = new_headers
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
    durations = [duration*counter_precision for duration in durations]

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

    if std_dev > 10:
      print(
        "%%%%%%%%%%%%%%%%\n"
        "% std_dev > 10 %\n"
        "%%%%%%%%%%%%%%%%"
        )
      exit(-1)

  process_duration = time.time() - process_time

  # print statistics if we computed them
  if compute_stats:
    print("  average: {:.2f}us, dev: {:5.2f}us, min: {:5.2f}us,"
      " max: {:5.2f}us, rate: {:.2f}kHz".format(
      average, std_dev, min, max, rate*1000))

  # print it for information
  print("{}/{}: waited: {:4.1f}ms, transfer: {:3.0f}us, process:{:4.1f}ms, pulses: {}"
    .format(line, packet_id, wait_duration*1e3, transfer_duration*1e6,
    process_duration*1e3, pulses[line]))
