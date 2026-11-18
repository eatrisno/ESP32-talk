#!/usr/bin/env python3
"""
ESP32-S3 HT Full Duplex Mesh Walkie-Talkie - Interactive Console
Memudahkan pemantauan log dan kontrol perangkat dari komputer.
"""
import sys
import time
import glob
import serial
import threading

def find_serial_ports():
    ports = glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*")
    return sorted(list(set(ports)))

def main():
    ports = find_serial_ports()
    if not ports:
        print("[ERROR] Tidak ditemukan port serial ESP32 (/dev/cu.usbmodem*).")
        print("Pastikan kabel USB ESP32-S3 telah terhubung.")
        return

    print("==================================================")
    print(" ESP32-S3 HT FULL DUPLEX MESH - TEST CONSOLE     ")
    print("==================================================")
    print("Port serial terdeteksi:")
    for idx, p in enumerate(ports):
        print(f"  [{idx + 1}] {p}")

    chosen_port = ports[0]
    if len(ports) > 1:
        sel = input("Pilih port [default 1]: ").strip()
        if sel.isdigit() and 1 <= int(sel) <= len(ports):
            chosen_port = ports[int(sel) - 1]

    print(f"\nMenghubungkan ke: {chosen_port} (115200 baud)...")
    try:
        ser = serial.Serial(chosen_port, 115200, timeout=0.1)
    except Exception as e:
        print(f"[ERROR] Gagal membuka port: {e}")
        return

    print("Terhubung! Membaca log perangkat...\n")
    print("--------------------------------------------------")
    print("KONTROL CEPAT (Ketik huruf lalu tekan Enter):")
    print("  t / Space -> Toggle Bicara / Join Voice (HT Full Duplex)")
    print("  + / u     -> Volume UP (+10%)")
    print("  - / d     -> Volume DOWN (-10%)")
    print("  p         -> Fleet Discovery Ping (Cek Jangkauan Mesh)")
    print("  c         -> Auto-Search Saluran RF (1..13)")
    print("  ?         -> Tampilkan status & menu")
    print("  q         -> Keluar dari console")
    print("--------------------------------------------------\n")

    running = True

    def reader_thread():
        while running:
            try:
                line = ser.readline().decode('utf-8', errors='replace')
                if line:
                    sys.stdout.write(line)
                    sys.stdout.flush()
            except Exception:
                break

    t = threading.Thread(target=reader_thread, daemon=True)
    t.start()

    time.sleep(0.3)
    ser.write(b"?\n")

    try:
        while running:
            cmd = input().strip()
            if not cmd:
                continue
            if cmd.lower() in ['q', 'exit', 'quit']:
                break
            ser.write((cmd + "\n").encode('utf-8'))
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        ser.close()
        print("\nKoneksi serial ditutup. Selesai.")

if __name__ == '__main__':
    main()
