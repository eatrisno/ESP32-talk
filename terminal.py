#!/usr/bin/env python3
"""
terminal.py - Dual-Node ESP32-S3 Mesh Monitor & Automated Test Suite

Usage:
    python3 terminal.py
    python3 terminal.py --port1 /dev/cu.usbmodem... --port2 /dev/cu.usbmodem...

Commands in interactive prompt:
    <cmd>        Send command to BOTH nodes (e.g. 's', 'scan', 'debug', 'walk')
    1: <cmd>     Send command to Node 1 only (e.g. '1: ch 11')
    2: <cmd>     Send command to Node 2 only (e.g. '2: p')
    b: <cmd>     Send command to BOTH nodes
    test-switch  Run automated channel propagation test (switches Node 1, measures Node 2 sync)
    test-ping    Run automated mesh ping test
    test-status  Compare status of both nodes
    quit / exit  Exit monitor
"""

import sys
import os
import time
import glob
import threading
import argparse
import readline

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Error: pyserial is required. Install with: pip install pyserial")
    sys.exit(1)

# ANSI Colors
CYAN = "\033[96m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
MAGENTA = "\033[95m"
RED = "\033[91m"
BOLD = "\033[1m"
DIM = "\033[2m"
RESET = "\033[0m"

class NodeMonitor:
    def __init__(self, node_id, port, baud=115200, color=CYAN):
        self.node_id = node_id
        self.port = port
        self.baud = baud
        self.color = color
        self.ser = None
        self.running = False
        self.thread = None
        self.callbacks = []
        self.name = f"Node-{node_id}"

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.running = True
            self.thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"{RED}[{self.name}] Failed to connect to {self.port}: {e}{RESET}")
            return False

    def disconnect(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass

    def send(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                payload = (cmd.strip() + "\n").encode("utf-8")
                self.ser.write(payload)
                self.ser.flush()
            except Exception as e:
                print(f"{RED}[{self.name}] TX Error: {e}{RESET}")
        else:
            print(f"{RED}[{self.name}] Cannot send, port not open!{RESET}")

    def add_callback(self, cb):
        self.callbacks.append(cb)

    def remove_callback(self, cb):
        if cb in self.callbacks:
            self.callbacks.remove(cb)

    def _reader_loop(self):
        buf = ""
        while self.running:
            try:
                data = self.ser.read(128)
                if not data:
                    continue
                text = data.decode("utf-8", errors="replace")
                buf += text
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip("\r")
                    self._handle_line(line)
            except Exception as e:
                if self.running:
                    print(f"{RED}[{self.name}] Disconnected: {e}{RESET}")
                break
        self.disconnect()

    def _handle_line(self, line):
        if not line:
            return

        # Notify test listeners
        for cb in list(self.callbacks):
            try:
                cb(self.node_id, line)
            except Exception:
                pass

        # Colorize important events
        prefix = f"{self.color}{BOLD}[Node-{self.node_id}]{RESET} "
        highlight = line

        if "[CHANNEL SYNC]" in line:
            highlight = f"{MAGENTA}{BOLD}{line}{RESET}"
        elif "[AUTO-JOIN]" in line:
            highlight = f"{YELLOW}{BOLD}{line}{RESET}"
        elif "[WATCHDOG]" in line:
            highlight = f"{RED}{BOLD}{line}{RESET}"
        elif "[AUTO-CONSENSUS]" in line or "CONSENSUS DECISION" in line:
            highlight = f"{CYAN}{BOLD}{line}{RESET}"
        elif "[DISTRESS" in line:
            highlight = f"{RED}{BOLD}{line}{RESET}"
        elif "probe #" in line:
            highlight = f"{GREEN}{line}{RESET}"

        timestamp = time.strftime("%H:%M:%S")
        sys.stdout.write(f"{DIM}{timestamp}{RESET} {prefix}{highlight}\n")
        sys.stdout.flush()


def find_esp32_ports():
    ports = []
    # Check macOS usbmodem
    for p in sorted(glob.glob("/dev/cu.usbmodem*")):
        ports.append(p)
    # Check Linux ACM
    for p in sorted(glob.glob("/dev/ttyACM*")):
        ports.append(p)
    # Fallback to serial.tools.list_ports
    if not ports:
        for p in list_ports.comports():
            if "usbmodem" in p.device.lower() or "usb" in p.description.lower():
                ports.append(p.device)
    return ports


class TestSuite:
    def __init__(self, node1, node2):
        self.node1 = node1
        self.node2 = node2

    def test_switch_propagation(self, target_ch=6):
        if not self.node1 or not self.node2:
            print(f"{RED}Error: Both nodes must be connected for propagation test!{RESET}")
            return

        print(f"\n{BOLD}=== TEST: Channel Propagation from Node 1 -> Node 2 ==={RESET}")
        print(f"Commanding Node 1 to switch to Channel {target_ch}...")

        propagated = threading.Event()
        start_time = 0
        sync_latency = [0]

        def on_line(node_id, line):
            nonlocal start_time
            if node_id == 2 and ("[CHANNEL SYNC]" in line and f"Primary Ch {target_ch}" in line):
                sync_latency[0] = (time.time() - start_time) * 1000
                propagated.set()

        self.node2.add_callback(on_line)
        start_time = time.time()
        self.node1.send(f"ch {target_ch}")

        success = propagated.wait(timeout=4.0)
        self.node2.remove_callback(on_line)

        if success:
            print(f"{GREEN}{BOLD}>>> PASS: Node 2 synced to Channel {target_ch} in {sync_latency[0]:.1f} ms! <<<{RESET}\n")
        else:
            print(f"{RED}{BOLD}>>> TIMEOUT: Node 2 did not sync to Channel {target_ch} within 4 seconds! <<<{RESET}\n")

    def test_ping(self):
        if not self.node1:
            print(f"{RED}Error: Node 1 must be connected!{RESET}")
            return

        print(f"\n{BOLD}=== TEST: Mesh Ping ==={RESET}")
        pong_received = threading.Event()

        def on_line(node_id, line):
            if "pong" in line.lower():
                pong_received.set()

        self.node1.add_callback(on_line)
        self.node1.send("p")
        success = pong_received.wait(timeout=3.0)
        self.node1.remove_callback(on_line)

        if success:
            print(f"{GREEN}{BOLD}>>> PASS: Ping acknowledged! <<<{RESET}\n")
        else:
            print(f"{YELLOW}>>> Ping sent (awaiting response) <<<{RESET}\n")


def print_help():
    print(f"""
{BOLD}=== Walkie-Talkie Multi-Node Console Commands ==={RESET}
  {CYAN}<cmd>{RESET}          Send command to BOTH nodes (e.g. 's', 'debug', 'walk', 'scan')
  {CYAN}1: <cmd>{RESET}       Send command to Node 1 only (e.g. '1: ch 11' or '1: s')
  {CYAN}2: <cmd>{RESET}       Send command to Node 2 only (e.g. '2: scan')
  {CYAN}test-switch [N]{RESET} Test auto channel propagation (default Ch 6 or N)
  {CYAN}test-ping{RESET}      Test radio ping round-trip
  {CYAN}debug{RESET}          Toggle quiet mode vs verbose debug alerts
  {CYAN}s{RESET}              Print status on both nodes
  {CYAN}h / help{RESET}       Show this help
  {CYAN}quit / exit{RESET}    Exit monitor
""")


def main():
    parser = argparse.ArgumentParser(description="ESP32-S3 Dual Node Mesh Monitor")
    parser.add_argument("--port1", help="Serial port for Node 1")
    parser.add_argument("--port2", help="Serial port for Node 2")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    args = parser.parse_args()

    ports = find_esp32_ports()
    port1 = args.port1 or (ports[0] if len(ports) > 0 else None)
    port2 = args.port2 or (ports[1] if len(ports) > 1 else None)

    print(f"{BOLD}=== ESP32-S3 Multi-Node Mesh Monitor & Test Suite ==={RESET}")
    if port1:
        print(f"  {CYAN}Node 1:{RESET} {port1}")
    else:
        print(f"  {RED}Node 1: None detected{RESET}")

    if port2:
        print(f"  {GREEN}Node 2:{RESET} {port2}")
    else:
        print(f"  {YELLOW}Node 2: Not connected yet (will auto-detect when plugged in){RESET}")

    node1 = NodeMonitor(1, port1, args.baud, CYAN) if port1 else None
    node2 = NodeMonitor(2, port2, args.baud, GREEN) if port2 else None

    if node1:
        node1.connect()
    if node2:
        node2.connect()

    tests = TestSuite(node1, node2)
    print_help()

    # Dynamic port watcher thread for Node 2
    def port_watcher():
        nonlocal node2
        while True:
            time.sleep(2.0)
            if node2 is None or not node2.running:
                cur_ports = find_esp32_ports()
                for p in cur_ports:
                    if node1 and p == node1.port:
                        continue
                    print(f"\n{GREEN}{BOLD}[AUTO-DETECT] Found Node 2 on {p}! Connecting...{RESET}\n")
                    node2 = NodeMonitor(2, p, args.baud, GREEN)
                    if node2.connect():
                        tests.node2 = node2
                        break

    watcher_thread = threading.Thread(target=port_watcher, daemon=True)
    watcher_thread.start()

    while True:
        try:
            cmd = input(f"{BOLD}> {RESET}").strip()
            if not cmd:
                continue

            if cmd in ("quit", "exit"):
                break
            elif cmd in ("h", "help", "?"):
                print_help()
            elif cmd.startswith("test-switch"):
                parts = cmd.split()
                ch = int(parts[1]) if len(parts) > 1 else 6
                tests.test_switch_propagation(ch)
            elif cmd == "test-ping":
                tests.test_ping()
            elif cmd.startswith("1:") or cmd.startswith("1 "):
                actual_cmd = cmd[2:].strip()
                if node1:
                    node1.send(actual_cmd)
                else:
                    print(f"{RED}Node 1 not connected{RESET}")
            elif cmd.startswith("2:") or cmd.startswith("2 "):
                actual_cmd = cmd[2:].strip()
                if node2:
                    node2.send(actual_cmd)
                else:
                    print(f"{RED}Node 2 not connected{RESET}")
            elif cmd.startswith("b:") or cmd.startswith("b "):
                actual_cmd = cmd[2:].strip()
                if node1: node1.send(actual_cmd)
                if node2: node2.send(actual_cmd)
            else:
                # Default: send to both
                if node1: node1.send(cmd)
                if node2: node2.send(cmd)

        except (KeyboardInterrupt, EOFError):
            print("\nExiting...")
            break

    if node1: node1.disconnect()
    if node2: node2.disconnect()
    print("Done.")


if __name__ == "__main__":
    main()
