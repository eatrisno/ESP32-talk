#!/usr/bin/env python3
"""
collector.py - Stream ESP32-S3 Walkie-Talkie range, RSSI & voltage telemetry to InfluxDB 2 + Grafana.

Usage:
    python3 monitoring/collector.py
    python3 monitoring/collector.py --port /dev/cu.usbmodem5C830796231 --label 15m
    python3 monitoring/collector.py --headless --label walk-test

Interactive commands while streaming:
    tag <label>   Set InfluxDB test label (e.g. 'tag 15m', 'tag ch6-outdoor', 'tag live')
    walk / range 20 / s / sig / ch 6 / ...   Forwarded straight to the ESP32-S3 console
    quit / exit   Stop collector
"""

import argparse
import glob
import json
import re
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

try:
    import serial
except ImportError:
    print("pyserial not found in this Python. Run with PlatformIO python:")
    print("  ~/.platformio/penv/bin/python monitoring/collector.py")
    sys.exit(1)

PROBE_OK_RE = re.compile(
    r"probe\s+#(\d+)\s+from\s+\[([^\]]+)\]:\s+OK\s+\|\s*(\d+)%\s+Link\s+\|\s*"
    r"RSSI:\s*(-?\d+)\s+dBm\s+\(est\.\s*~([\d.]+)\s*(cm|m)\)\s*\|\s*"
    r"Uplink:\s*(-?\d+)\s+dBm"
)
PROBE_DROP_RE = re.compile(r"probe\s+#(\d+)\s*:\s*\[(?:DROPPED|OUT OF RANGE)")
RANGE_SUMMARY_LOSS_RE = re.compile(r"Packet Loss:\s*([\d.]+)%")
RANGE_HDR_RE = re.compile(r"STARTING RANGE TEST.*@\s+(.+?)\s+Ch\s+(\d+)")


def escape_tag(val: str) -> str:
    return str(val).strip().replace("\\", "\\\\").replace(" ", "\\ ").replace(",", "\\,").replace("=", "\\=") or "unknown"


class InfluxWriter:
    def __init__(self, url: str, org: str, bucket: str, token: str):
        self.write_url = (
            f"{url.rstrip('/')}/api/v2/write?"
            + urllib.parse.urlencode({"org": org, "bucket": bucket, "precision": "ns"})
        )
        self.token = token
        self._warned = False

    def write_lines(self, lines: list[str]):
        if not lines:
            return
        payload = ("\n".join(lines) + "\n").encode("utf-8")
        req = urllib.request.Request(
            self.write_url,
            data=payload,
            method="POST",
            headers={
                "Authorization": f"Token {self.token}",
                "Content-Type": "text/plain; charset=utf-8",
            },
        )
        try:
            with urllib.request.urlopen(req, timeout=5.0) as resp:
                if resp.status not in (200, 204) and not self._warned:
                    print(f"[influx] unexpected HTTP {resp.status}", file=sys.stderr)
                    self._warned = True
                self._warned = False
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            if not self._warned:
                print(f"[influx] write failed ({e}). Is Docker stack up? ('docker compose up -d')", file=sys.stderr)
                self._warned = True


class NodeCollector:
    def __init__(self, port: str, baud: int, writer: InfluxWriter, label_ref: list[str], poll_s: float):
        self.port = port
        self.baud = baud
        self.writer = writer
        self.label_ref = label_ref
        self.poll_s = poll_s
        self.ser = None
        self._tx_lock = threading.Lock()
        self.running = False
        self.range_active = False
        self.node_name = port.split("/")[-1]
        self.channel = "13"
        self.rate = "LR_250k"
        self.last_peer = "unknown"

    def start(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        time.sleep(0.4)
        self.running = True
        threading.Thread(target=self._read_loop, daemon=True).start()
        if self.poll_s > 0:
            threading.Thread(target=self._poll_loop, daemon=True).start()

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, cmd: str):
        with self._tx_lock:
            if self.ser and self.ser.is_open:
                self.ser.write((cmd.strip() + "\n").encode("utf-8"))
                self.ser.flush()

    def _poll_loop(self):
        while self.running:
            time.sleep(self.poll_s)
            if not self.range_active:
                try:
                    self.send("telem")
                except Exception:
                    pass

    def _read_loop(self):
        buf = ""
        while self.running:
            try:
                chunk = self.ser.read(512)
                if not chunk:
                    continue
                buf += chunk.decode("utf-8", errors="replace")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip("\r")
                    if line:
                        self._process_line(line)
            except Exception as e:
                if self.running:
                    print(f"[{self.port}] serial error: {e}", file=sys.stderr)
                break

    def _process_line(self, line: str):
        now_ns = time.time_ns()
        label = escape_tag(self.label_ref[0])

        if line.startswith("TELEM "):
            try:
                data = json.loads(line[6:])
                self._emit_telem(data, now_ns, label)
            except json.JSONDecodeError:
                pass
            return

        # Print non-TELEM lines for operator visibility
        print(line)

        hdr = RANGE_HDR_RE.search(line)
        if hdr:
            self.range_active = True
            self.rate = hdr.group(1).strip()
            self.channel = hdr.group(2)
            return
        if "RANGE TEST SUMMARY" in line or "NO LINK" in line or "=========================================================" in line:
            self.range_active = False

        m_ok = PROBE_OK_RE.search(line)
        if m_ok:
            peer = m_ok.group(2).strip()
            self.last_peer = peer
            link_pct = int(m_ok.group(3))
            dl_rssi = int(m_ok.group(4))
            est_val = float(m_ok.group(5))
            est_m = est_val / 100.0 if m_ok.group(6) == "cm" else est_val
            ul_rssi = int(m_ok.group(7))
            asym = ul_rssi - dl_rssi
            margin = dl_rssi - (-108.0)
            lp = (
                f"rf_link,node={escape_tag(self.node_name)},peer={escape_tag(peer)},"
                f"ch={escape_tag(self.channel)},rate={escape_tag(self.rate)},source=probe,label={label} "
                f"downlink_rssi={dl_rssi}i,uplink_rssi={ul_rssi}i,asymmetry_db={asym}i,"
                f"margin_db={margin:.1f},link_pct={link_pct}i,est_m={est_m:.2f},loss_pct=0.0 {now_ns}"
            )
            self.writer.write_lines([lp])
            return

        if PROBE_DROP_RE.search(line):
            lp = (
                f"rf_link,node={escape_tag(self.node_name)},peer={escape_tag(self.last_peer)},"
                f"ch={escape_tag(self.channel)},rate={escape_tag(self.rate)},source=probe,label={label} "
                f"loss_pct=100.0 {now_ns}"
            )
            self.writer.write_lines([lp])

    def _emit_telem(self, d: dict, now_ns: int, label: str):
        self.node_name = d.get("node") or self.node_name
        self.channel = str(d.get("ch", self.channel))
        self.rate = str(d.get("rate", self.rate))
        node_tag = escape_tag(self.node_name)
        id_tag = escape_tag(d.get("id", "0"))
        ch_tag = escape_tag(self.channel)
        rate_tag = escape_tag(self.rate)

        lines = []
        fields = [
            f"temp_c={float(d.get('temp_c', 0.0)):.2f}",
            f"tx_dbm={float(d.get('tx_dbm', 0.0)):.2f}",
            f"tx={int(d.get('tx', 0))}i",
            f"rx={int(d.get('rx', 0))}i",
            f"tx_fail={int(d.get('tx_fail', 0))}i",
            f"rx_drop={int(d.get('rx_drop', 0))}i",
            f"relayed={int(d.get('relayed', 0))}i",
            f"audio_lost={int(d.get('audio_lost', 0))}i",
            f"audio_underrun={int(d.get('audio_underrun', 0))}i",
        ]
        volt_mv = int(d.get("volt_mv", 0))
        if volt_mv > 0:
            fields.append(f"voltage_v={volt_mv / 1000.0:.3f}")
        lines.append(
            f"node_health,node={node_tag},node_id={id_tag},role=local,ch={ch_tag},label={label} "
            + ",".join(fields)
            + f" {now_ns}"
        )

        for p in d.get("peers", []):
            pname = escape_tag(p.get("name", "unknown"))
            self.last_peer = p.get("name", self.last_peer)
            pid = escape_tag(p.get("id", "0"))
            dl = int(p.get("rssi", 0))
            ul = int(p.get("up_rssi", 0))
            hops = int(p.get("hops", 0))
            p_volt = int(p.get("volt_mv", 0))

            if dl != 0:
                margin = dl - (-108.0)
                link_fields = [
                    f"downlink_rssi={dl}i",
                    f"margin_db={margin:.1f}",
                    f"hops={hops}i",
                ]
                if ul != 0:
                    link_fields.append(f"uplink_rssi={ul}i")
                    link_fields.append(f"asymmetry_db={ul - dl}i")
                lines.append(
                    f"rf_link,node={node_tag},peer={pname},ch={ch_tag},rate={rate_tag},source=telem,label={label} "
                    + ",".join(link_fields)
                    + f" {now_ns}"
                )
            if p_volt > 0:
                lines.append(
                    f"node_health,node={pname},node_id={pid},role=peer,ch={ch_tag},label={label} "
                    f"voltage_v={p_volt / 1000.0:.3f} {now_ns}"
                )

        self.writer.write_lines(lines)


def find_ports() -> list[str]:
    return sorted(glob.glob("/dev/cu.usbmodem*") or glob.glob("/dev/ttyACM*"))


def main():
    ap = argparse.ArgumentParser(description="ESP32-S3 Walkie-Talkie InfluxDB Telemetry Collector")
    ap.add_argument("--port", help="Serial port (default: auto-detect first usbmodem)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--influx-url", default="http://localhost:8086")
    ap.add_argument("--org", default="walkie-talkie")
    ap.add_argument("--bucket", default="rf_telemetry")
    ap.add_argument("--token", default="wt-local-dev-token-2026")
    ap.add_argument("--label", default="live", help="Initial test label tag in Grafana (e.g. '15m', 'desk')")
    ap.add_argument("--poll", type=float, default=2.0, help="Seconds between telem polls (0 = disable)")
    ap.add_argument("--headless", action="store_true", help="Run without interactive stdin prompt")
    args = ap.parse_args()

    port = args.port
    if not port:
        ports = find_ports()
        if not ports:
            print("No ESP32-S3 serial ports found (/dev/cu.usbmodem*).", file=sys.stderr)
            sys.exit(1)
        port = ports[0]
        print(f"Auto-selected port: {port}")

    label_ref = [args.label]
    writer = InfluxWriter(args.influx_url, args.org, args.bucket, args.token)
    col = NodeCollector(port, args.baud, writer, label_ref, args.poll)
    col.start()
    print(f"Streaming telemetry from {port} -> {args.influx_url} (bucket={args.bucket}, label={label_ref[0]})")
    print("Grafana: http://localhost:3030  |  Commands: 'tag <label>', 'walk', 'range 20', 's', 'quit'")

    try:
        if args.headless:
            while True:
                time.sleep(1)
        else:
            while True:
                line = input().strip()
                if line in ("quit", "exit"):
                    break
                if line.startswith("tag "):
                    new_lbl = line[4:].strip() or "live"
                    label_ref[0] = new_lbl
                    print(f"[collector] label set to: {new_lbl}")
                    continue
                if line:
                    col.send(line)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        col.stop()


if __name__ == "__main__":
    main()
