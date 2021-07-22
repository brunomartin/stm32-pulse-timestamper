# This script listen uint32_t timestamps array on UDP port
# To avoid UDP packet mixing, it must be executed on the network
# "near" the pulse timestamper, e.g., not on wifi

import socket
import struct
import time
import math
import argparse
import queue
import os

from record_thread import RecordThreadText, RecordThreadBinary, RecordThreadBinaryLog, RecordThreadBinaryRotated

parser = argparse.ArgumentParser()

parser.add_argument("-s", "--statistics",
                    help="compute statistics",
                    action="store_true",
                    default=False)

parser.add_argument("-r", "--record",
                    help="record timestamps",
                    action="store_true",
                    default=False)

parser.add_argument("-p", "--port",
                    help="port where timestamps are sent",
                    default=8042)

args = parser.parse_args()

UDP_IP = ""
UDP_PORT = args.port

lines = 2

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP
sock.bind((UDP_IP, UDP_PORT))

# UDP packet sizes
fragment_header_size = 12
fragment_data_size = 1000  # 250x4

# UDP packet fragment count
fragment_count = 4

# timestamp max in unit
counter_period = 4000000000

# timestamp precision in us
counter_precision = 1.0  # 1MHz
counter_precision = 0.1  # 10MHz
counter_precision = 0.0125  # 80MHz

# minimum rate to detect in Hz, if period is higher, continguous
# timestamp analysis is reset
max_wait_duration_s = 2.0

# Tell if we want to compute statistics or not
compute_stats = args.statistics
# compute_stats = True

# Tell if we record timestamps
record_timetamps = args.record
# record_timetamps = True

print("port: {}".format(UDP_PORT))
print("compute_stats: {}".format(compute_stats))
print("record_timetamps: {}".format(record_timetamps))

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
last_timestamps = []

last_packet_id = []

wait_duration_start = []
first_fragment_time = []

pulses = []

for i in range(lines):
    last_timestamps.append([])
    last_packet_id.append(-1)
    wait_duration_start.append(time.time())
    pulses.append(0)

record_queue = queue.Queue()
# record_thread = RecordThreadText(record_queue, lines)
# record_thread = RecordThreadBinary(record_queue, lines)
# record_thread = RecordThreadBinaryLog(record_queue, lines)
record_thread = RecordThreadBinaryRotated(record_queue, lines)

if record_timetamps:
    record_thread.start()
    print("os.getgid(): {}".format(os.getgid()))
    # print("record_thread.pid: {}".format(record_thread.pid))

try:
    while True:

        record_duration = 0

        for fragment_index in range(0, fragment_count):

            # Receive and store whole fragment
            fragment, addr = sock.recvfrom(fragment_size)

            fragment_header = fragment[:fragment_header_size]
            fragment_data = fragment[fragment_header_size:]

            # extract header from fragment
            if header_size > 0:
                header_start = fragment_index * fragment_header_size
                header_end = header_start + fragment_header_size
                header[header_start:header_end] = fragment_header

            # extract data from fragment
            data_start = fragment_index * fragment_data_size
            data_end = data_start + fragment_data_size
            data[data_start:data_end] = fragment_data

            if fragment_index == 0:
                first_fragment_time = time.time()

            fragment_record_time = time.time()
            if record_timetamps:
                message = fragment_header, fragment_data
                record_queue.put_nowait(message)

                if record_queue.qsize() > 200:
                    print("record_queue.qsize(): {}".format(record_queue.qsize()))
                    exit(-1)

            fragment_record_duration = time.time() - fragment_record_time
            record_duration += fragment_record_duration

        # unpack header content : line, packet_id, fragment_id
        headers = list(struct.unpack('3I', header[0:12]))

        line = headers[0]
        packet_id = headers[1]

        # compute timings involved
        transfer_end_time = time.time()
        wait_duration = transfer_end_time - wait_duration_start[line]
        wait_duration_start[line] = transfer_end_time
        transfer_duration = transfer_end_time - first_fragment_time

        # if we waited too long, throw away last timestamps for all lines
        if wait_duration > max_wait_duration_s:
            last_timestamps[0:lines] = [[] for i in range(lines)]
            last_packet_id[0:lines] = [-1 for i in range(lines)]
            wait_duration = 0

        process_time = time.time()

        # convert data to uint32 array
        new_timestamps = list(struct.unpack('{}I'.format(int(len(data) / 4)), data))

        # Detect discontinuation in packet ids
        if last_packet_id[line] == -1:
            last_packet_id[line] = packet_id
        else:
            if packet_id != last_packet_id[line] + 1:
                # print(fragment)
                # print(headers)
                # print(new_timestamps)
                print(
                    "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
                    "% packet_id ({}) != last_packet_id + 1 ({}) %\n"
                    "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
                    .format(packet_id, last_packet_id[line] + 1))
                exit(-1)

            last_packet_id[line] = packet_id

        # remove trailing magic value if any
        new_timestamps = [x for x in new_timestamps if x < counter_period]

        # Increment total number of pulses from script start
        pulses[line] += len(new_timestamps)

        if compute_stats:

            # concatenate with last timestamps to ensure continuity
            timestamps = last_timestamps[line] + new_timestamps
            last_timestamps[line] = new_timestamps

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

            def compute_statistics(values):
                average = 0
                min = values[0]
                max = values[0]
                for value in values:
                    average += value
                    min = value if value < min else min
                    max = value if value > max else max
                average /= len(values)

                std_dev = 0
                for duration in values:
                    std_dev += (duration - average)**2
                std_dev /= len(values)
                std_dev = math.sqrt(std_dev)

                return average, std_dev, min, max

            # compute statistics
            average, std_dev, min, max = compute_statistics(durations)

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

            compute_delay = (lines > 1) and (line == 1)
            for i in range(lines):
                compute_delay &= len(last_timestamps[i]) > 0

            if compute_delay:
                # compute delay between line 1 pulse and line 0 pulse
                delays = [(last_timestamps[1][i] - last_timestamps[0][i]) for
                  i in range(len(last_timestamps))]

                delays = [delay*counter_precision for delay in delays]

                delay_average, delay_std_dev, delay_min, delay_max = compute_statistics(delays)

        process_duration = time.time() - process_time

        # print statistics if we computed them
        if compute_stats:
            print("  average: {:.2f}us, dev: {:5.2f}us, min: {:5.2f}us,"
                " max: {:5.2f}us, rate: {:.2f}kHz".format(
                average, std_dev, min, max, rate*1000))

            if compute_delay:
                print("  delay: average: {:.2f}us, dev: {:5.2f}us, min: {:5.2f}us,"
                    " max: {:5.2f}us".format(
                delay_average, delay_std_dev, delay_min, delay_max))

        else:  # compute stats
            average = new_timestamps[-1] - new_timestamps[0]
            average /= len(new_timestamps) - 1
            average *= counter_precision
            rate = 1/average if average != 0 else math.nan
            print("  average: {:.2f}us, rate: {:.2f}kHz".format(
              average, rate*1000))

        # print it for information
        print("{}/{}: waited: {:4.1f}ms, Tx: {:3.0f}us, proc: {:4.1f}ms, rec: {:4.1f}ms, pulses: {}"
            .format(line, packet_id, wait_duration*1e3, transfer_duration*1e6,
            process_duration*1e3, record_duration*1e3, pulses[line]))

except (KeyboardInterrupt, SystemExit):
    print("\n! Received keyboard interrupt, quitting threads.\n")

# Wait for recording thread if it is running
if record_timetamps:
    record_thread.stop()
