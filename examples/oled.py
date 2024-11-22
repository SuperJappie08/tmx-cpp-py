import atexit
from datetime import timedelta

import tmx_cpp_py


def nop():
    print("Telemtrix cleanup on too large ping diff")
    return


if __name__ == "__main__":
    tmx = tmx_cpp_py.TMX(nop)
    atexit.register(tmx.stop)

    tmx.send_message(tmx_cpp_py.MessageType.GET_PICO_UNIQUE_ID, [])
    tmx.set_scan_delay(100)

    assert tmx.connected

    modules = tmx_cpp_py.Modules(tmx)

    assert tmx.set_i2c_pins(4, 5, 0)

    oled = tmx_cpp_py.SSD1306_Module(0, 0x3C)

    modules.add_mod(oled)

    oled.send_text("Booting...", timeout=timedelta(seconds=4))

    input()
