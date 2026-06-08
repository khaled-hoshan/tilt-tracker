import asyncio
import json
import serial
import serial.tools.list_ports
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from contextlib import asynccontextmanager

# ── Global state ───────────────────────────────────────────────────────
# Holds the most recent reading from the STM32.
# Shape matches exactly what the firmware sends.
latest = {
    "tr": 0.0,    # trig roll
    "tp": 0.0,    # trig pitch
    "mr": 0.0,    # madgwick roll
    "mp": 0.0,    # madgwick pitch
    "my": 0.0,    # madgwick yaw
}

# List of currently connected browser WebSocket clients.
# When a new reading arrives from the STM32 we broadcast to all of them.
clients: list[WebSocket] = []


def find_stm32_port() -> str:
    """
    Auto-detect which serial port the Black Pill is on.
    STMicroelectronics USB vendor ID is 0483.
    This avoids hardcoding /dev/ttyACM0 which might change.
    """
    for port in serial.tools.list_ports.comports():
        if "0483" in port.hwid:
            return port.device
    return "/dev/ttyACM0"    # Fallback if auto-detect fails


async def serial_reader():
    """
    Background task that runs for the entire lifetime of the server.

    Architecture (mini pub/sub):
      STM32 (producer) → serial port → this function (broker)
                                              ↓
                                     WebSocket clients (consumers)

    We run the blocking serial.readline() in a thread pool executor
    so it doesn't block the asyncio event loop. This lets FastAPI
    handle WebSocket connections at the same time.
    """
    global latest

    port = find_stm32_port()
    print(f"[serial] Opening {port} at 115200 baud")
    ser = serial.Serial(port, baudrate=115200, timeout=1)

    loop = asyncio.get_event_loop()

    while True:
        try:
            # readline() blocks until \n arrives — run in thread pool
            raw = await loop.run_in_executor(None, ser.readline)
            line = raw.decode("utf-8").strip()

            if not line:
                continue

            data = json.loads(line)

            # Skip non-data messages like {"status":"ready"}
            if "tr" not in data:
                continue

            latest = data

            # Broadcast to every connected browser
            disconnected = []
            for ws in clients:
                try:
                    await ws.send_json(data)
                except Exception:
                    disconnected.append(ws)    # Client disconnected

            for ws in disconnected:
                clients.remove(ws)

        except json.JSONDecodeError:
            pass    # Ignore incomplete packets during startup/reset

        except Exception as e:
            print(f"[serial] Error: {e}")
            await asyncio.sleep(1)    # Brief pause before retrying


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Start serial reader when server starts, stop it when server stops."""
    task = asyncio.create_task(serial_reader())
    yield
    task.cancel()


app = FastAPI(title="Tilt Tracker", lifespan=lifespan)


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    Each browser tab that opens the dashboard connects here.
    We add the WebSocket to the clients list so serial_reader()
    can push new data to it whenever a packet arrives from the STM32.
    """
    await websocket.accept()
    clients.append(websocket)
    try:
        # Send the latest reading immediately so the page isn't blank
        await websocket.send_json(latest)
        # Keep the connection alive until the browser disconnects
        while True:
            await asyncio.sleep(30)
    except WebSocketDisconnect:
        clients.remove(websocket)


@app.get("/api/latest")
async def get_latest():
    """Simple REST endpoint — useful for debugging or testing."""
    return latest


# Static file server for the frontend.
# Must be mounted LAST so it doesn't shadow the API routes above.
app.mount("/", StaticFiles(directory="../frontend", html=True), name="static")