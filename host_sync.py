from datetime import timedelta
MS_THRESHOLD = 5

class HostSync:
    def __init__(self):
        self.arrays = {}

    def add_msg(self, name, msg):
        if not name in self.arrays:
            self.arrays[name] = []

        self.arrays[name].append({"msg": msg, "seq": msg.getSequenceNum()})

        synced = {}
        for name, arr in self.arrays.items():
            for i, obj in enumerate(arr):
                if msg.getSequenceNum() == obj["seq"]:
                    synced[name] = obj["msg"]
                    break

        if len(synced) == 4:
            for name, arr in self.arrays.items():
                for i, obj in enumerate(arr):
                    if obj["seq"] < msg.getSequenceNum():
                        arr.remove(obj)
                    else:
                        break
            return synced
        return False

    def add_msg_t(self, name, msg, ts = None):
        if ts is None:
            ts = msg.getTimestampDevice()

        if not name in self.arrays:
            self.arrays[name] = []

        self.arrays[name].append((ts, msg))

        synced = {}
        for name, arr in self.arrays.items():
            # Go through all stored messages and calculate the time difference to the target msg.
            # Then sort these msgs to find a msg that's closest to the target time, and check
            # whether it's below 17ms which is considered in-sync.
            diffs = []
            for i, (msg_ts, msg) in enumerate(arr):
                diffs.append(abs(msg_ts - ts))
            if len(diffs) == 0: break
            diffsSorted = diffs.copy()
            diffsSorted.sort()
            dif = diffsSorted[0]

            if dif < timedelta(milliseconds=MS_THRESHOLD):
                # print(f'Found synced {name} with ts {msg_ts}, target ts {ts}, diff {dif}, location {diffs.index(dif)}')
                # print(diffs)
                synced[name] = diffs.index(dif)


        if len(synced) == 4: # We have 3 synced msgs (IMU packet + disp + rgb)
            # print('--------\Synced msgs! Target ts', ts, )
            # Remove older msgs
            for name, i in synced.items():
                self.arrays[name] = self.arrays[name][i:]
            ret = {}
            for name, arr in self.arrays.items():
                ret[name] = arr.pop(0)[1]
                # print(f'{name} msg ts: {ret[name][0]}, diff {abs(ts - ret[name][0]).microseconds / 1000}ms')
            return ret
        return False