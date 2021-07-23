import struct
import threading
import socket


class Packet:
    def __init__(self, fragments_per_packet):
        self.fragments_per_packet = fragments_per_packet
        self.id = -1
        self.fragment_ids = []
        self.header_list = []
        self.data_list = []

    def check(self):
        packet_ok = True

        packet_ok &= len(self.fragment_ids) == self.fragments_per_packet

        return packet_ok

    def clear(self):
        self.fragment_ids = []
        self.header_list = []
        self.data_list = []

    def empty(self):
        return len(self.fragment_ids) == 0

    def get_data(self):
        data = bytes()
        for i in range(len(self.fragment_ids)):
            data += self.data_list[self.fragment_ids[i]]

        return data


# Base class for recorder thread
class ReceiveThread(threading.Thread):
    def __init__(self, port=8042, header_size=12, data_size=1000,
                 fragments_per_packet=4, lines=2, record_queue=None,
                 statistic_queue=None):

        super(ReceiveThread, self).__init__()

        self.port = port
        self.header_size = header_size
        self.data_size = data_size
        self.fragments_per_packet = fragments_per_packet
        self.record_queue = record_queue
        self.statistic_queue = statistic_queue
        self.lines = lines
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__stop = False

        self.packets = []
        for i in range(self.lines):
            self.packets.append(Packet(self.fragments_per_packet))

        self.last_packet_ids = []
        for i in range(self.lines):
            self.last_packet_ids.append(-1)

    def start(self):
        self.__socket.bind(("", self.port))
        self.__socket.settimeout(0.5)
        super(ReceiveThread, self).start()

    def stop(self):
        self.__stop = True
        self.__socket.close()
        self.join()

    def __del__(self):
        pass

    def run(self):

        fragment_size = self.header_size + self.data_size

        while not self.__stop:
            # Receive and store whole fragment

            # socket.timeout will be send if no data arrived in time
            # OSError if socket has been closed when this thread is stopping
            try:
                fragment, addr = self.__socket.recvfrom(fragment_size)
            except socket.timeout:
                continue
            except OSError:
                continue

            header = fragment[:self.header_size]
            data = fragment[self.header_size:]

            # Check if fragment id is consistent with current index
            line, packet_id, fragment_id = list(struct.unpack('{}I'.format(3), header))

            packet = self.packets[line]
            last_packet_id = self.last_packet_ids[line]

            # Check packet consistency
            if packet.empty():
                # Check if we lost a packet
                if last_packet_id == -1:
                    last_packet_id = packet_id - 1

                if packet_id != last_packet_id + 1:
                    print("packet_id != last_packet_id + 1")
                    exit(-1)

                packet.id = packet_id
            else:
                # If a packet is currently being retrieved
                # check if same packet
                if packet_id != packet.id:
                    print("packet_id != packet.id")
                    exit(-1)

            packet.fragment_ids.append(fragment_id)
            packet.header_list.append(header)
            packet.data_list.append(data)

            if packet.check():

                if self.record_queue:
                    message = packet.header_list[0], packet.get_data()
                    self.record_queue.put_nowait(message)

                    # Check if there is no contention in record queue
                    if self.record_queue.qsize() > 200:
                        print("record_queue.qsize(): {}".format(self.record_queue.qsize()))
                        exit(-1)

                if self.statistic_queue:
                    message = line, packet.get_data()
                    self.statistic_queue.put_nowait(message)

                packet.clear()
