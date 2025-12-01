import datetime
import os
from enum import Enum
from flask import Flask, request, send_file
import requests

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR = os.path.join(BASE_DIR, "logs")

WTM_OTA_PATH = os.path.join(BASE_DIR, "WTMFirmware.bin")
WTM_Z41_OTA_PATH = os.path.join(BASE_DIR, "WTMFirmwareZ41.bin")
WTM_SLAVE_OTA_PATH = os.path.join(BASE_DIR, "WTMFirmwareSlave.bin")
WTM_MASTER_OTA_PATH = os.path.join(BASE_DIR, "WTMFirmwareMaster.bin")

app = Flask(__name__)

class Status(Enum):
    ONLINE = "Online"
    OFFLINE = "Offline"

server_status = Status.OFFLINE


def get_log_file(entity: str, entity_id: int = None) -> str:
    """Return log file path inside logs/YYYY-MM-DD/ folder."""
    date_dir = os.path.join(LOGS_DIR, datetime.date.today().strftime("%Y-%m-%d"))
    os.makedirs(date_dir, exist_ok=True)

    if entity == "infeed":
        filename = "infeed.txt"
    elif entity == "wtm":
        filename = f"wtm_{entity_id}.txt"
    elif entity == "bot":
        filename = f"bot_{entity_id}.txt"
    else:
        raise ValueError("Invalid entity")

    return os.path.join(date_dir, filename)


def write_log(entity: str, data: str, entity_id: int = None):
    """Append log entry to the correct file."""
    filepath = get_log_file(entity, entity_id)
    with open(filepath, "a", encoding="utf-8") as f:
        f.write(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S}: {data}\n")
        
    # text_file_path = os.path.join(LOGS_DIR, datetime.date.today().strftime("%Y-%m-%d") + "espData.txt")
    # with open(text_file_path, "a") as file:
    #     file.write(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S}:   {data}\n")


@app.route("/log/infeed", methods=["POST"])
def log_infeed():
    data = request.get_data(as_text=True).strip()
    if not data:
        return "no data", 400
    write_log("infeed", data)
    return "ok"


@app.route("/log/wtm/<string:wtm_id>", methods=["POST"])
def log_wtm(wtm_id):
    data = request.get_data(as_text=True).strip()
    if not data:
        return "no data", 400
    write_log("wtm", data, wtm_id)
    return "ok"


@app.route("/log/bot/<string:bot_id>", methods=["POST"])
def log_bot(bot_id):
    data = request.get_data(as_text=True).strip()
    if not data:
        return "no data", 400
    write_log("bot", data, bot_id)
    return "ok"


@app.route("/download_WTM")
def download_wtm_firmware():
    return send_file(WTM_OTA_PATH, as_attachment=True)

@app.route("/download_WTM_slave")
def download_wtm_firmware_slave():
    return send_file(WTM_SLAVE_OTA_PATH, as_attachment=True)

@app.route("/download_WTM_master")
def download_wtm_firmware_master():
    return send_file(WTM_MASTER_OTA_PATH, as_attachment=True)

@app.route("/download_WTM_Z41")
def download_wtm_firmware_z41():
    return send_file(WTM_Z41_OTA_PATH, as_attachment=True)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
