import datetime
import os
from flask import Flask, request, send_file

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR = os.path.join(BASE_DIR, "logs")

WTM_OTA_PATH = os.path.join(BASE_DIR, "WTMFirmware.bin")
WTM_Z41_OTA_PATH = os.path.join(BASE_DIR, "WTMFirmwareZ41.bin")
WTM_SLAVE_OTA_PATH = os.path.join(BASE_DIR, "WTMFirmwareSlave.bin")
WTM_MASTER_OTA_PATH = os.path.join(BASE_DIR, "WTMFirmwareMaster.bin")

app = Flask(__name__)


def get_log_file(entity: str, entity_id: str = None) -> str:
    """Return log file path inside logs/YYYY-MM-DD/entity_type/ folder."""
    date_dir = os.path.join(LOGS_DIR, datetime.date.today().strftime("%Y-%m-%d"))
    entity_dir = os.path.join(date_dir, entity)
    os.makedirs(entity_dir, exist_ok=True)

    if entity_id:
        filename = f"{entity_id}-logs.txt"
    else:
        filename = f"{entity}-logs.txt"

    return os.path.join(entity_dir, filename)


def write_log(entity: str, data: str, entity_id: str = None):
    """Append log entry to the correct file."""
    filepath = get_log_file(entity, entity_id)
    with open(filepath, "a", encoding="utf-8") as f:
        f.write(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S}: {data}\n")


@app.route("/log", methods=["POST"])
def log_data():
    entity = request.args.get("entity")
    entity_id = request.args.get("entity_id")
    
    if not entity:
        return "entity is required", 400
    
    data = request.get_data(as_text=True).strip()
    if not data:
        return "no data", 400
    
    write_log(entity, data, entity_id)
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
