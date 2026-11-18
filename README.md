# ESP32-S3 HT Full Duplex Mesh Walkie-Talkie

Perangkat Handy Talkie (HT) digital mandiri (*headless*), terenkripsi, dan beroperasi secara *off-grid* berbasis **ESP32-S3**. Dirancang khusus untuk komunikasi suara dua arah secara simultan (**Full Duplex**) melalui jaringan **ESP-NOW Long-Range Mesh**, menggunakan headset standar USB Type-C tanpa membutuhkan layar, internet, maupun router Wi-Fi.

---

## ⚡ Fitur Utama (Core HT Features)

```
┌────────────────────────────────┐         Encrypted ESP-NOW LR          ┌────────────────────────────────┐
│      ESP32-S3 Node: "Peto"     │◄─────────────────────────────────────►│      ESP32-S3 Node: "Bubu"     │
│  • Pure Station (WIFI_STA)     │         Channels 1–13 (Auto-Sync)     │  • Pure Station (WIFI_STA)     │
│  • +20 dBm TX Power (100mW)    │         AES-128 Hardware Crypto       │  • +20 dBm TX Power (100mW)    │
│  • 50ms Live Voice Streaming   │         Voice + Discovery Ping        │  • 50ms Live Voice Streaming   │
│  • Type-C Headset (Mic & Spk)  │          Autonomous Mesh Relay        │  • Type-C Headset (Mic & Spk)  │
│  • HT Full Duplex Audio Mixer  │                                       │  • HT Full Duplex Audio Mixer  │
└────────────────────────────────┘                                       └────────────────────────────────┘
                                                ▲
                                                │ Mesh Relay (TTL=3)
                                                ▼
                                 ┌────────────────────────────────┐
                                 │      ESP32-S3 Node: "Wawa"     │
                                 │  • Pure Station (WIFI_STA)     │
                                 │  • Autonomous Packet Forwarder │
                                 │  • Real-Time Audio Engine      │
                                 └────────────────────────────────┘
```

1. **Jaringan Mesh ESP-NOW (Autonomous Store-and-Forward Relay):**
   - Protokol ESP-NOW Long-Range (`WIFI_PROTOCOL_LR`) dengan daya pancar maksimum +20 dBm (100 mW).
   - PHY rate 6 Mbps OFDM (airtime ultra-singkat ~340 µs/paket) untuk meminimalkan risiko tabrakan RF.
   - Fitur repeater otonom antar node dengan batasan `TTL = 3` (jangkauan dapat menembus halangan hingga 1.5–2+ km).
   - Buffer deduplikasi 64-entri untuk mencegah *broadcast storm* dan *packet loop*.
   - Keamanan enkripsi perangkat keras **AES-128-CTR** pada setiap paket kontrol dan suara.

2. **HT Full Duplex & Multi-Party Audio Mixer:**
   - Komunikasi dua arah simultan: pengguna dapat berbicara dan mendengarkan suara teman secara bersamaan tanpa terpotong (*no channel lockout*).
   - Digital mixer audio real-time dengan proteksi saturasi sinyal:
     $$\text{Sample}_{\text{mixed}} = \text{clamp}\big(\text{Sample}_A + \text{Sample}_B, -32768, 32767\big)$$
   - Codec suara efisien IMA-ADPCM 4:1 (kompresi 16-bit 8 kHz ke 32 kbps).
   - Jitter buffer sirkular (8 slot $\times$ 50ms) dengan prebuffering 100ms yang berjalan di FreeRTOS Core 1 untuk output suara yang mulus tanpa putus-putus (*zero stutter*).
   - Filter DSP *80 Hz DC-blocker* untuk menghilangkan noise napas dan letupan angin.

3. **Tombol Headset / PTT Press untuk Join:**
   - Tekan tombol headset (atau GPIO 4 / GPIO 0) untuk **JOIN** ke saluran suara (*Full Duplex Active*).
   - Tekan kembali untuk **LEAVE** / *Mute* kembali ke mode siaga (*Standby*).
   - Dilengkapi nada notifikasi akustik ke telinga (*ascending chime* saat Join, *descending chime* saat Leave).

4. **Pengatur Volume Fisik (Up & Down):**
   - Tombol fisik Volume UP (GPIO 1) dan Volume DOWN (GPIO 2).
   - Pengaturan skala volume 0% – 100% (langkah 10%).
   - Umpan balik nada suara langsung ke earphone (nada tinggi saat volume naik, nada rendah saat volume turun, nada batas saat 100% atau 0%).
   - Level volume tersimpan permanen di memori flash NVS (*non-volatile storage*).

5. **Sinkronisasi Saluran & Penemuan Node (Auto-Discovery):**
   - Deteksi otomatis saluran RF 1–13 jika belum terhubung dengan node rekan.
   - *Fleet Discovery Ping*: memeriksa node rekan yang aktif dan mengukur latensi round-trip (RTT) serta jalur hop (*Direct* vs *Mesh Relay*).

---

## 🔌 Skema Pinout & Pengkabelan Hardware

| ESP32-S3 Pin | Fungsi | Keterangan Sambungan |
|---|---|---|
| **GPIO 19** | USB OTG D- | Terhubung ke jalur D- port USB Type-C earphone |
| **GPIO 20** | USB OTG D+ | Terhubung ke jalur D+ port USB Type-C earphone |
| **GPIO 4** | **Headset / External Button** | Tombol Headset / tombol eksternal untuk **Join/Leave Voice** (`INPUT_PULLUP` ke GND) |
| **GPIO 0** | **BOOT Button** | Tombol pada board DevKit (Klik: Join/Leave, Double Klik: Cari Saluran) |
| **GPIO 1** | **Volume UP Button** | Tombol fisik untuk menaikkan volume (`INPUT_PULLUP` ke GND) |
| **GPIO 2** | **Volume DOWN Button** | Tombol fisik untuk menurunkan volume (`INPUT_PULLUP` ke GND) |
| **GPIO 48** | WS2812 RGB LED | Lampu indikator status on-board |
| **GPIO 21** | Buzzer | Output buzzer alert (3.3V aktif/pasif) |
| **5V / VBUS** | Daya Headset | Sumber tegangan 5V ke port USB Type-C headset |

> [!IMPORTANT]
> Headset USB Type-C digital memerlukan suplai daya 5V pada pin VBUS port Type-C. Pastikan jalur 5V ESP32 terhubung ke pin VBUS Type-C atau gunakan splitter/OTG adapter yang memiliki injeksi daya 5V.

---

## 🎛️ Panduan Penggunaan Tombol

| Tombol / Aksi | Fungsi | Indikator LED | Umpan Balik Nada Headset |
|---|---|---|---|
| **Tekan Tombol Headset (GPIO 4)** | **Toggle Join / Leave Saluran Suara** | **Merah Solid** saat Join, **Biru Singkat** saat Leave | *Ascending Chime* (Join) / *Descending Chime* (Leave) |
| **Tekan Tombol BOOT (GPIO 0)** | **Toggle Join / Leave** (Identik dengan Headset) | **Merah Solid** saat Join | *Ascending Chime* / *Descending Chime* |
| **Double-Click BOOT (<350ms)** | **Auto-Search Saluran** (Scan Ch 1–13) | **Kuning/Amber Berkedip** | Nada konfirmasi ganda saat saluran terkunci |
| **Tekan Tombol Vol UP (GPIO 1)** | **Menaikkan Volume (+10%)** | — | Beep frekuensi tinggi (1500 Hz / limit 2200 Hz) |
| **Tekan Tombol Vol DOWN (GPIO 2)** | **Menurunkan Volume (-10%)** | — | Beep frekuensi rendah (750 Hz / limit 400 Hz) |

---

## 💡 Arti Warna Lampu Indikator LED (WS2812 RGB)

| Warna LED | Status Perangkat |
|---|---|
| 🔴 **Merah Solid** | **Pengirim Aktif (Joined):** Mikrofon sedang merekam dan mentransmisikan suara Full Duplex |
| 🟢 **Hijau Solid / Kedip** | **Penerima:** Suara dari node rekan sedang masuk dan diputar ke headset / Node terdeteksi |
| 🔵 **Biru Singkat** | **Konfirmasi Transmisi:** Selesai mentransmisikan paket suara / kembali ke Standby |
| 🟠 **Kuning / Amber** | **Pencarian Saluran:** Sedang memindai saluran RF 1 hingga 13 |
| 🩵 **Cyan** | **Ping:** Paket ping sedang dikirim / Saluran RF berhasil terkunci |
| ⚪ **Mati (Off)** | **Siaga (Standby):** Siap menerima transmisi suara rekan kapan saja |

---

## 🛠️ Panduan Build & Flash Firmware

### 1. Prasyarat
- [PlatformIO](https://platformio.org/) (terpasang via CLI atau ekstensi VSCode/Cursor)
- Python 3.8+

### 2. Kompilasi Firmware
Buka terminal pada direktori `esp32-walkie-talkie` dan jalankan:
```bash
python3 -m platformio run
```

### 3. Flash ke Perangkat ESP32-S3
Hubungkan board ESP32-S3 ke komputer menggunakan kabel data USB (port UART/USB DevKit):
```bash
python3 -m platformio run -t upload
```

### 4. Buka Terminal Pemantau Interaktif
Gunakan skrip Python bawaan untuk memantau status perangkat dan mengirim perintah:
```bash
python3 scripts/interactive_terminal.py
```
Atau menggunakan serial monitor PlatformIO (115200 baud):
```bash
python3 -m platformio device monitor -b 115200
```

### 5. Hubungkan Node Kedua
Lakukan langkah yang sama pada unit ESP32-S3 kedua. Kedua perangkat akan otomatis saling menemukan melalui jaringan ESP-NOW Mesh dan siap digunakan untuk komunikasi suara Full Duplex!
