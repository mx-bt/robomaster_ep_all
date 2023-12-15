import time
import robomaster
from robomaster import conn
from MyQR import myqr
from PIL import Image

QRCODE_NAME = "qrcode.png"
helper = conn.ConnectionHelper()
info = helper.build_qrcode_string(ssid="wifi_name", password="wifi_access_key")
myqr.run(words=info)
time.sleep(1)
img = Image.open(QRCODE_NAME)
img.show()
