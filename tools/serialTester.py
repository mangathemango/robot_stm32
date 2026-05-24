import serial
import serial.tools.list_ports
import threading
import struct

BAUD_RATE  = 115200
START_BYTE = 0x67

# ── Incoming packet IDs (STM32 -> PC) ────────────────────────────────────────
INCOMING = {
    0x50: "Log",
    0x51: "WheelVelocities",
    0x52: "Key1",
    0x53: "HorizontalArmPosition",
    0x54: "VerticalArmPosition",
}

# ── Outgoing packet IDs (PC -> STM32) ────────────────────────────────────────
OUTGOING = {
    "1": (0x01, "Set_Yaw_Servo_Angle"),
    "2": (0x02, "Set_Claw_Servo_Angle"),
    "3": (0x03, "Set_Display_Text"),
    "6": (0x06, "Set_Vertical_Arm_Position"),
    "7": (0x07, "Set_Horizontal_Arm_Position"),
    "8": (0x08, "Beep"),
    "9": (0x09, "Set_Wheel_Target_Velocities"),
}

# ── Packet builder ────────────────────────────────────────────────────────────
def build_packet(cmd_id: int, data: bytes) -> bytes:
    header = bytes([START_BYTE, cmd_id, len(data)]) + data
    checksum = 0
    for b in header:
        checksum ^= b
    return header + bytes([checksum])

def encode_outgoing(cmd_id: int, name: str):
    """Prompt for arguments and return a ready-to-send packet, or None to cancel."""

    # 0x01 / 0x02 — single angle byte (0-180)
    if cmd_id in (0x01, 0x02):
        val = int(input("  Angle (0-180): "))
        return build_packet(cmd_id, bytes([val & 0xFF]))

    # 0x03 — ASCII string
    if cmd_id == 0x03:
        text = input("  Text: ").encode("ascii", errors="replace")
        return build_packet(cmd_id, text)

    # 0x06 / 0x07 — uint16 position (0-10000)
    if cmd_id in (0x06, 0x07):
        val = int(input("  Position (0-10000): "))
        return build_packet(cmd_id, struct.pack("<H", val))

    # 0x08 — no data
    if cmd_id == 0x08:
        return build_packet(cmd_id, b"")

    # 0x09 — 4x int16 wheel velocities
    if cmd_id == 0x09:
        vfl = int(input("  vfl (-10000 to 10000): "))
        vfr = int(input("  vfr (-10000 to 10000): "))
        vrl = int(input("  vrl (-10000 to 10000): "))
        vrr = int(input("  vrr (-10000 to 10000): "))
        return build_packet(cmd_id, struct.pack("<hhhh", vfl, vfr, vrl, vrr))

    return None

# ── Packet decoder ────────────────────────────────────────────────────────────
def decode_incoming(cmd_id: int, data: bytes) -> str:
    """Return a human-readable string for a received packet."""

    # 0x50 — Log string
    if cmd_id == 0x50:
        return f'Log: "{data.decode("ascii", errors="replace")}"'

    # 0x51 — 4x int16 wheel velocities
    if cmd_id == 0x51 and len(data) == 8:
        vfl, vfr, vrl, vrr = struct.unpack("<hhhh", data)
        return f"WheelVelocities: vfl={vfl} vfr={vfr} vrl={vrl} vrr={vrr}"

    # 0x52 — Key1 (no data)
    if cmd_id == 0x52:
        return "Key1 pressed"

    # 0x53 — HorizontalArmPosition uint16
    if cmd_id == 0x53 and len(data) == 2:
        pos = struct.unpack("<H", data)[0]
        return f"HorizontalArmPosition: {pos}"

    # 0x54 — VerticalArmPosition uint16
    if cmd_id == 0x54 and len(data) == 2:
        pos = struct.unpack("<H", data)[0]
        return f"VerticalArmPosition: {pos}"

    return f"Unknown ID=0x{cmd_id:02X} data={data.hex()}"

# ── Receive thread ────────────────────────────────────────────────────────────
def receive_loop(ser: serial.Serial):
    """
    Binary packet reader.
    State machine: waits for START_BYTE, reads ID + LEN, reads LEN data bytes,
    reads checksum, validates, then decodes.
    """
    while True:
        try:
            # Sync to start byte
            b = ser.read(1)
            if not b or b[0] != START_BYTE:
                continue

            # Read ID + LEN
            header = ser.read(2)
            if len(header) < 2:
                continue
            cmd_id, length = header[0], header[1]

            # Read data + checksum
            rest = ser.read(length + 1)
            if len(rest) < length + 1:
                continue
            data     = rest[:length]
            received = rest[length]

            # Validate checksum
            computed = START_BYTE ^ cmd_id ^ length
            for byte in data:
                computed ^= byte

            if computed != received:
                print(f"\n[RX] Checksum error on ID=0x{cmd_id:02X} "
                      f"(got 0x{received:02X}, expected 0x{computed:02X})")
                print("Send> ", end="", flush=True)
                continue

            # Decode and print
            msg = decode_incoming(cmd_id, data)
            print(f"\n[STM32] {msg}")
            print("Send> ", end="", flush=True)

        except serial.SerialException:
            print("\n[ERROR] Serial connection lost.")
            break

# ── Port selection ────────────────────────────────────────────────────────────
def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No COM ports found. Make sure your STM32 is connected.")
        return []
    print("\nAvailable COM ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} — {p.description}")
    return ports

# ── Send menu ─────────────────────────────────────────────────────────────────
def print_menu():
    print("\n┌─ Commands (PC -> STM32) ───────────────────────────┐")
    for key, (cid, name) in OUTGOING.items():
        print(f"│  {key}  →  {name:<39}│")
    print("│  q  →  Quit                                       │")
    print("└────────────────────────────────────────────────────┘")

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    ports = list_ports()
    if not ports:
        return

    if len(ports) == 1:
        chosen = ports[0]
        print(f"\nAuto-selected: {chosen.device}")
    else:
        idx = int(input("\nEnter port number: "))
        chosen = ports[idx]

    print(f"Connecting to {chosen.device} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(chosen.device, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"[ERROR] Could not open port: {e}")
        return

    print("Connected!\n")

    try:
        while True:
            print_menu()
            choice = input("Send> ").strip().lower()

            if choice == "q":
                break

            if choice not in OUTGOING:
                print("  Invalid choice.")
                continue

            cmd_id, name = OUTGOING[choice]
            print(f"  Building packet: {name}")
            try:
                packet = encode_outgoing(cmd_id, name)
                if packet:
                    ser.write(packet)
                    print(f"  Sent {len(packet)} bytes: {packet.hex(' ').upper()}")
            except (ValueError, struct.error) as e:
                print(f"  [ERROR] Bad input: {e}")

    except KeyboardInterrupt:
        pass
    finally:
        print("\nExiting.")
        ser.close()

if __name__ == "__main__":
    main()