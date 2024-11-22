import atexit
from datetime import timedelta
from pathlib import Path

import numpy as np
import tmx_cpp_py
from PIL import Image

if __name__ == "__main__":
    tmx = tmx_cpp_py.TMX()
    atexit.register(tmx.stop)

    tmx.send_message(tmx_cpp_py.MessageType.GET_PICO_UNIQUE_ID, [])
    tmx.set_scan_delay(100)

    assert tmx.connected

    modules = tmx_cpp_py.Modules(tmx)

    assert tmx.set_i2c_pins(4, 5, 0)

    oled = tmx_cpp_py.SSD1306_Module(0, 0x3C)

    modules.add_mod(oled)

    image_path = Path(__file__).parent / "mirte_logo_inv.png"

    image = Image.open(image_path)
    image = image.convert("L", dither=Image.NONE)

    data = np.array(image)

    oled.send_image(
        image.width,
        image.height,
        data.flatten().astype(int),
        timeout=timedelta(seconds=4),
    )

    input()
