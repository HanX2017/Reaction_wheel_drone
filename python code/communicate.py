import serial
import time
import asyncio
import keyboard  # 改用 keyboard 監聽鍵盤
from bleak import BleakClient
import logging
from datetime import datetime

# 產生唯一的 log 檔案名稱
log_filename = f"output_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

# 設定 logging，確保同時輸出到檔案與終端
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(message)s"
)

# 清除預設 handler，避免重複輸出
for handler in logging.root.handlers[:]:
    logging.root.removeHandler(handler)

# 建立檔案與終端輸出
file_handler = logging.FileHandler(log_filename, encoding="utf-8")
console_handler = logging.StreamHandler()

# 設定格式
formatter = logging.Formatter("%(asctime)s - %(message)s")
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

# 設定 Logger
logger = logging.getLogger()
logger.addHandler(file_handler)
logger.addHandler(console_handler)


# === 設定 Arduino 串列通訊 ===
ARDUINO_PORT = "COM5"  # 修改為你的 Arduino 端口
BAUDRATE_ARDUINO = 9600

# === 設定 ESP32 BLE 參數 ===
ESP32_MAC = "F8:B3:B7:44:54:16"  # 修改為你的 ESP32 BLE MAC 地址
CHARACTERISTIC_UUID = "87654321-4321-4321-4321-cba987654321"  # 修改為你的 UUID

# 建立 Serial 連接
ser_arduino = serial.Serial(ARDUINO_PORT, BAUDRATE_ARDUINO, timeout=1)
connect_flag = False

time.sleep(2)  # 等待串口連線穩定

# 初始化 BLE 客戶端
client = None

async def connect_ble():
    """初始化並連接 BLE 服務"""
    global client
    client = BleakClient(ESP32_MAC)
    try:
        await client.connect()
    except Exception as e:
        logging.info(f"Error while connecting to ESP32: {e}")

async def send_message_to_esp32(message):
    """發送訊息到 ESP32"""
    if client and client.is_connected:
        try:
            await client.write_gatt_char(CHARACTERISTIC_UUID, message.encode(), response=False)
        except Exception as e:
            logging.info(f"Error while sending message to ESP32: {e}")
            await reconnect_ble()  
    else:
        logging.info("ESP32 disconnected, attempting to reconnect...")
        await reconnect_ble()

async def reconnect_ble():
    """重新連接到 ESP32"""
    global client
    logging.info("Reconnecting to ESP32...")
    await client.connect()
    if client.is_connected:
        logging.info("Reconnected to ESP32.")
    else:
        logging.info("Failed to reconnect to ESP32.")

async def receive_message_from_esp():
    """從 ESP32 接收訊息"""
    async def notification_handler(sender, data):
        logging.info(data.decode()    )

    if client and client.is_connected:
        try:
            await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
            logging.info("ESP32 Notify 啟動成功")
        except Exception as e:
            logging.info(f"啟用 Notify 失敗: {e}")

def receive_message_from_arduino():
    """接收 Arduino 訊息"""
    messages = []
    while ser_arduino.in_waiting > 0:
        message = ser_arduino.readline().decode(errors='ignore').strip()
        if message:
            messages.append(message)
    return messages if messages else False

async def handle_arduino():
    """處理 Arduino 訊息並發送到 ESP32"""
    msg_ard_list = receive_message_from_arduino()
    if msg_ard_list:        
        for msg_ard in msg_ard_list:
            await send_message_to_esp32(msg_ard)

async def handle_keyboard():
    """偵測空白鍵按下，並發送訊息"""
    while True:
        if keyboard.is_pressed("space"):
            await send_message_to_esp32("SPACE_PRESSED")
            await asyncio.sleep(0.3)
        if keyboard.is_pressed("enter"):
            await send_message_to_esp32("ENTER_PRESSED")
            await asyncio.sleep(0.3)
        for key, message in zip("qweasdrtyfghujikol", ["KPX_PLUS", "KIX_PLUS", "KDX_PLUS", "KPX_MINUS", "KIX_MINUS", "KDX_MINUS", "KPY_PLUS", "KIY_PLUS", "KDY_PLUS", "KPY_MINUS", "KIY_MINUS", "KDY_MINUS", "XA_PLUS", "XA_MINUS", "YA_PLUS", "YA_MINUS", "S_PLUS", "S_MINUS"]):
            if keyboard.is_pressed(key): 
                await send_message_to_esp32(message)
                await asyncio.sleep(0.3)
        await asyncio.sleep(0.01)

async def main():
    global connect_flag
    try:
        await connect_ble()
        await receive_message_from_esp()
        asyncio.create_task(handle_keyboard())

        while True:
            await handle_arduino()
            if not connect_flag:
                ser_arduino.write("online".encode())
                connect_flag = True
            if client and not client.is_connected:
                logging.info("ESP32 disconnected, attempting to reconnect...")
                connect_flag = False
                ser_arduino.write("offline".encode())
                await reconnect_ble()
                await receive_message_from_esp()
            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        ser_arduino.close()
        if client:
            await client.disconnect()   
        logging.info("Serial and BLE connections closed.")
    except Exception as e:
        logging.info(f"Error: {e}")
        ser_arduino.close()
        if client:
            await client.disconnect()

# 執行
asyncio.run(main())
