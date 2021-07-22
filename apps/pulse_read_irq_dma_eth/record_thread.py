# This script listen uint32_t timestamps array on UDP port
# To avoid UDP packet mixing, it must be executed on the network
# "near" the pulse timestamper, e.g., not on wifi

import struct
import threading
import queue


# Base class for recorder thread
class RecordThread(threading.Thread):
    def __init__(self, queue, lines, *args, **kwargs):
        super(RecordThread, self).__init__(*args, **kwargs)
        self.queue = queue
        self.lines = lines
        self.files = []
        self.__stop = False

    def start(self):
        self.open_files()
        self.__stop = False
        super(RecordThread, self).start()

    def stop(self):
        self.__stop = True
        self.join()

    def open_files(self):
        raise NotImplementedError()

    def close_files(self):
        raise NotImplementedError()

    def __del__(self):
        self.close_files()

    def record(self, line, packet_id, fragment_id, timestamps):
        raise NotImplementedError()

    def run(self):
        while not self.__stop:
            # Wait for item in queue, if none after 0.5 seconds,
            # continue to check if program is stopped
            try:
                header, data = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue

            # Extract metada and data from header and data
            line, packet_id, fragment_id = list(struct.unpack('{}I'.format(3), header))
            timestamps = list(struct.unpack('{}I'.format(int(len(data) / 4)), data))

            # Record it to file, this one must be open and writable
            self.record(line, packet_id, fragment_id, timestamps)


# Simple text file recorder
class RecordThreadText(RecordThread):
    def __init__(self, *args, **kwargs):
        super(RecordThreadText, self).__init__(*args, **kwargs)

    def open_files(self):
        if len(self.files) > 0:
            raise Exception("Files already open")

        for line in range(self.lines):
            self.files.append(open("timestamps_" + str(line) + ".txt", "a"))

    def close_files(self):
        for file in self.files:
            file.close()
        self.files = []

    def record(self, line, packet_id, fragment_id, timestamps):
        for timestamp in timestamps:
            # line_str = str(packet_id) + "/" + str(fragment_id)
            # line_str += " : "
            line_str = str(timestamp)
            self.files[line].write(line_str + "\n")

