from collections import deque
import math


class StreamGuard:
    """Measure source time, age, rate and gaps; reject ROS reception-time restamping."""
    def __init__(self, min_rate, max_age, max_gap, window=2.):
        self.min_rate, self.max_age, self.max_gap, self.window = min_rate,max_age,max_gap,window
        self.samples=deque()
        self.last=None
        self.last_received=None
        self.error="no samples"

    def feed(self, stamp, now):
        if not math.isfinite(stamp) or stamp <= 0 or not -.02 <= now-stamp <= self.max_age:
            self.error="source timestamp stale/future/invalid"
            return False
        if self.last is not None and (stamp <= self.last or stamp-self.last > self.max_gap):
            self.error="source timestamp reversed/repeated/gap"
            self.samples.clear()
            # Recovery requires a new complete observation window.
            self.last=stamp
            return False
        self.last=stamp
        self.last_received=now
        self.samples.append(stamp)
        while self.samples and stamp-self.samples[0] > self.window:
            self.samples.popleft()
        self.error=""
        return True

    def ready(self, now):
        return (not self.error and len(self.samples) >= 3 and
                now-self.last <= self.max_age and
                self.samples[-1]-self.samples[0] >= self.window*.8 and
                (len(self.samples)-1)/(self.samples[-1]-self.samples[0]) >= self.min_rate)

    def rate(self):
        return (len(self.samples)-1)/(self.samples[-1]-self.samples[0]) if len(self.samples)>1 else 0.


class ClockGuard:
    def __init__(self, tolerance=.02):
        self.offset=None
        self.tolerance=tolerance
        self.failed=False

    def check(self, ros, mono):
        offset=ros-mono
        if self.offset is not None and abs(offset-self.offset) > self.tolerance:
            self.failed=True
        self.offset=offset
        return not self.failed
