"""Clock-domain mapping independent of ROS and Gazebo transport."""


class SourceClockMapper:
    """Map a source clock into the local ROS clock domain.

    Gazebo simulation timestamps commonly start at zero, while ROS nodes here
    use system time. The constant offset preserves the source sample spacing
    without mixing those clock epochs. A backward jump indicates a simulator
    clock reset, so the mapping is re-anchored at the current ROS time.
    """

    def __init__(self):
        self._offset_ns = None
        self._last_source_ns = None

    def to_ros_nanoseconds(self, source_stamp, ros_now_ns):
        source_ns = int(source_stamp[0]) * 1_000_000_000 + int(source_stamp[1])
        if self._offset_ns is None or (
            self._last_source_ns is not None
            and source_ns < self._last_source_ns
        ):
            self._offset_ns = int(ros_now_ns) - source_ns
        self._last_source_ns = source_ns
        return source_ns + self._offset_ns


class SensorDrivenClock:
    """Advance SITL by measured simulation time, never by packet arrivals."""

    def __init__(self):
        self.origin_ns = None
        self.last_ns = None

    def advance(self, source_ns):
        source_ns = int(source_ns)
        if self.last_ns is not None and source_ns <= self.last_ns:
            return None
        if self.origin_ns is None:
            self.origin_ns = source_ns
        self.last_ns = source_ns
        return .005 + (source_ns - self.origin_ns) * 1e-9
