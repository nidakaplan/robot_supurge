#!/usr/bin/env python3

import qrcode

data = "ROOM=LIVINGROOM"

img = qrcode.make(data)

img.save("qr_livingroom.png")
