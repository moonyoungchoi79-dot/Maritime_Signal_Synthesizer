import logging

APP_NAME = "Maritime Signal Synthesizer"
DEFAULT_WINDOW_SIZE = (1600, 900)

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("NMEA_Gen")

CSV_HEADER = [
    "id", "rx_time", "receiver_ship_index", "receiver_ship_name", "talker", "sentence_type", "raw"
]