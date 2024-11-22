import atexit
import time

import tmx_cpp_py

if __name__ == "__main__":
    tmx = tmx_cpp_py.TMX()
    atexit.register(tmx.stop)

    tmx.send_message(tmx_cpp_py.MessageType.GET_PICO_UNIQUE_ID, [])
    tmx.set_scan_delay(100)

    assert tmx.connected

    tmx.attach_servo(3)
    tmx.write_servo(3, 1000)
    time.sleep(1)
    tmx.write_servo(3, 2000)
