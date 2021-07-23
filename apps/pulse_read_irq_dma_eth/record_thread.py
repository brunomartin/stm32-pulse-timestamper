import struct
import threading
import queue

from pathlib import Path


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

    def record(self, header, data):
        raise NotImplementedError()

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

            self.record(header, data)


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

    def record(self, header, data):

        # Extract metadata and data from header and data
        line, packet_id, fragment_id = list(struct.unpack('{}I'.format(3), header))
        timestamps = list(struct.unpack('{}I'.format(int(len(data) / 4)), data))

        for timestamp in timestamps:
            line_str = str(timestamp)
            self.files[line].write(line_str + "\n")

        self.files[line].flush()


# Simple binary file recorder
# to print timestamp in integer one per line type the following command:
# hexdump -v -e '1/4 "%08u " "\n"' timestamps_1.bin
class RecordThreadBinary(RecordThreadText):
    def __init__(self, *args, **kwargs):
        super(RecordThreadBinary, self).__init__(*args, **kwargs)

    def open_files(self):
        if len(self.files) > 0:
            raise Exception("Files already open")

        for line in range(self.lines):
            file = open("timestamps_" + str(line) + ".bin", "ab")
            self.files.append(file)

    def record(self, header, data):
        line, _, _ = list(struct.unpack('{}I'.format(3), header))
        self.files[line].write(data)
        self.files[line].flush()


# Rotated text file recorder
class RecordThreadTextRotated(RecordThreadText):
    def __init__(self, *args, **kwargs):
        super(RecordThreadTextRotated, self).__init__(*args, **kwargs)

        self.current_files = []
        for line in range(self.lines):
            self.current_files.append(1)

    def open_files(self):
        if len(self.files) > 0:
            raise Exception("Files already open")

        for line in range(self.lines):

            self.current_files[line] = 1
            base_filename = "timestamps_" + str(line)
            while True:
                filename = base_filename + "_" + str(self.current_files[line]) + ".txt"

                if not Path(filename).exists():
                    self.current_files[line] -= 1
                    break

                self.current_files[line] += 1

            filename = base_filename + "_" + str(self.current_files[line]) + ".txt"
            file = open(filename, "a")
            self.files.append(file)

    def record(self, header, data):

        # Extract metadata and data from header and data
        line, packet_id, fragment_id = list(struct.unpack('{}I'.format(3), header))
        timestamps = list(struct.unpack('{}I'.format(int(len(data) / 4)), data))

        file = self.files[line]

        for timestamp in timestamps:
            line_str = str(timestamp)
            file.write(line_str + "\n")

        file.flush()

        if Path(file.name).stat().st_size >= 10*1024*1024:
            file.close()
            self.current_files[line] += 1
            filename = "timestamps_" + str(line)
            filename += "_" + str(self.current_files[line]) + ".txt"
            file = open(filename, "a")
            self.files[line] = file


# Rotated binary file recorder
# to print timestamp in integer one per line type the following command:
# hexdump -v -e '1/4 "%08u " "\n"' timestamps_0_0.bin
class RecordThreadBinaryRotated(RecordThreadText):
    def __init__(self, *args, **kwargs):
        super(RecordThreadBinaryRotated, self).__init__(*args, **kwargs)

        self.current_files = []
        for line in range(self.lines):
            self.current_files.append(1)

    def open_files(self):
        if len(self.files) > 0:
            raise Exception("Files already open")

        for line in range(self.lines):

            self.current_files[line] = 1
            base_filename = "timestamps_" + str(line)
            while True:
                filename = base_filename + "_" + str(self.current_files[line]) + ".bin"

                if not Path(filename).exists():
                    self.current_files[line] -= 1
                    break

                self.current_files[line] += 1

            filename = base_filename + "_" + str(self.current_files[line]) + ".bin"
            file = open(filename, "ab")
            self.files.append(file)

    def record(self, header, data):
        line, _, _ = list(struct.unpack('{}I'.format(3), header))

        file = self.files[line]

        file.write(data)
        file.flush()

        if Path(file.name).stat().st_size >= 10*1024*1024:
            file.close()
            self.current_files[line] += 1
            filename = "timestamps_" + str(line)
            filename += "_" + str(self.current_files[line]) + ".bin"
            file = open(filename, "ab")
            self.files[line] = file
