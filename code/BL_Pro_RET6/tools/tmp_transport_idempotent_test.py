import time

from debuglink_transport import DebugLinkTransport


PORT = "COM33"
BAUD = 921600


def main() -> None:
    t = DebugLinkTransport()
    frames: list[tuple[float, int]] = []
    t0 = time.time()

    def on_frame(frame) -> None:
        frames.append((time.time() - t0, frame.tick_ms))

    t.connect(PORT, BAUD)
    t.set_stream_callback(on_frame)

    print("start#1 100Hz ->", t.stream_start(100))
    time.sleep(1.2)
    n1 = len(frames)
    print("frames after 1.2s:", n1)

    print("start#2 100Hz ->", t.stream_start(100))
    time.sleep(1.2)
    n2 = len(frames) - n1
    print("new frames next 1.2s:", n2)

    print("switch to 50Hz ->", t.stream_start(50))
    time.sleep(1.6)
    n3 = len(frames) - n1 - n2
    print("new frames next 1.6s:", n3)

    print("stop ->", t.stream_stop())
    t.disconnect()


if __name__ == "__main__":
    main()

