import struct
import threading
import queue
import math
import time


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
        std_dev += (duration - average) ** 2
    std_dev /= len(values)
    std_dev = math.sqrt(std_dev)

    return average, std_dev, min, max


# Base class for recorder thread
class StatisticThread(threading.Thread):
    def __init__(self, queue, lines, counter_period=4000000000,
                 counter_precision=0.0125, max_timestamps=50000):

        super(StatisticThread, self).__init__()

        self.queue = queue
        self.lines = lines
        self.period = 1
        self.counter_period = counter_period
        self.counter_precision = counter_precision
        self.max_timestamps = max_timestamps
        self.__stop = False

        self.timestamps = []
        for i in range(lines):
            self.timestamps.append([])

        self.averages = []
        for i in range(lines):
            self.averages.append([])

        self.last_time = time.time()

    def start(self):
        super(StatisticThread, self).start()

    def stop(self):
        self.__stop = True
        self.join()

    def __del__(self):
        pass

    def run(self):

        while not self.__stop:
            # Wait for item in queue, if none after 0.5 seconds,
            # continue to check if program is stopped
            try:
                line, data = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue

            timestamps = self.timestamps[line]

            new_timestamps = list(struct.unpack('{}I'.format(int(len(data) / 4)), data))

            # remove trailing magic value if any
            new_timestamps = [x for x in new_timestamps if x < self.counter_period]

            # concatenate with last timestamps to ensure continuity
            timestamps += new_timestamps

            now = time.time()

            print_statistics = now - self.last_time >= self.period

            max_length = 0
            for i in range(self.lines):
                if len(self.timestamps[i]) > max_length:
                    max_length = len(self.timestamps[i])

            print_statistics |= max_length >= self.max_timestamps

            if print_statistics:

                for line in range(self.lines):
                    timestamps = self.timestamps[line]

                    if len(timestamps) < 2:
                        continue

                    # unwrap timestamps according to counter period
                    for i in range(1, len(timestamps)):
                        while timestamps[i] - timestamps[i-1] < -self.counter_period/2:
                            timestamps[i] += self.counter_period

                    # compute durations from timestamps
                    durations = [(timestamps[i] - timestamps[i-1])
                                 for i in range(1, len(timestamps)-1)]

                    # apply conversion according to precision
                    durations = [duration*self.counter_precision for duration in durations]

                    # compute statistics
                    self.averages[line], std_dev, min, max = compute_statistics(durations)

                    rate = math.nan
                    if self.averages[line] != 0:
                        rate = 1./self.averages[line]

                    print("{}: ({}k) av.: {:.2f}us, dev: {:5.2f}us, min: {:5.2f}us,"
                          " max: {:5.2f}us, rate: {:.2f}kHz".format(
                        line, round(len(timestamps)/1e3), self.averages[line],
                        std_dev, min, max, rate*1000))

                compute_delay = (self.lines > 1) and (line == 1)
                for line in range(self.lines):
                    compute_delay &= len(timestamps) > 0

                if compute_delay:
                    # compute delay between line 1 pulse and line 0 pulse

                    mid = int(len(self.timestamps[0])/2)

                    diff = [(self.timestamps[1][i] - self.timestamps[0][mid]) for
                              i in range(len(self.timestamps[1]))]

                    diff_min = abs(diff[0])
                    for shift in range(len(diff)):
                        if abs(diff[shift]) < abs(diff_min):
                            diff_min = diff[shift]

                    shift = diff.index(diff_min)
                    shift -= mid

                    min_length = len(self.timestamps[0])
                    for i in range(self.lines):
                        if len(self.timestamps[i]) < min_length:
                            min_length = len(self.timestamps[i])

                    if shift > 0:
                        delays = [(self.timestamps[1][i+shift] - self.timestamps[0][i]) for
                                  i in range(shift, min_length - shift)]
                        # print("test")
                    else:
                        delays = [(self.timestamps[1][i] - self.timestamps[0][i-shift]) for
                                  i in range(min_length + shift)]
                        # print("test")

                    delays = [delay*self.counter_precision for delay in delays]

                    delay_average = 0
                    delay_std_dev = 0
                    delay_min = 0
                    delay_max = 0

                    if len(delays) > 1:
                        delay_average, delay_std_dev, delay_min, delay_max = compute_statistics(delays)

                    print("  delay: average: {:.2f}us, dev: {:5.2f}us, min: {:5.2f}us,"
                          " max: {:5.2f}us".format(
                        delay_average, delay_std_dev, delay_min, delay_max))

                # Update line timestamps keeping last ones to
                # ensure continuity
                for line in range(self.lines):
                    self.timestamps[line] = self.timestamps[line][-1000:]

                self.last_time = now

            pass


