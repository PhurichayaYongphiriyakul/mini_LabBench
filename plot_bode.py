"""Trigger a Stage II sweep on the Nucleo and plot the Bode diagram.

Usage:  pip install pyserial matplotlib
        python plot_bode.py /dev/tty.usbmodemXXXX
"""
import sys
import csv
import serial
import matplotlib.pyplot as plt

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbmodem1103"
rows = []

with serial.Serial(port, 115200, timeout=120) as ser:
    ser.reset_input_buffer()
    ser.write(b"s")
    in_block = False
    while True:
        raw = ser.readline()
        if not raw:                      # nothing at all within the timeout
            sys.exit("Timeout - no data from board")
        line = raw.decode(errors="ignore").strip()
        if not line:                     # blank line from the board, keep reading
            continue
        print(line)
        if line == "BEGIN":
            in_block = True
        elif line == "END":
            break
        elif in_block and line.startswith("DATA,"):
            rows.append([float(v) for v in line.split(",")[1:]])

with open("stage2_bode.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["f_Hz", "gain", "gain_dB", "phase_deg",
                "vin_mVpp", "vout_mVpp", "vout_dc_mV", "clipped"])
    w.writerows(rows)

f     = [r[0] for r in rows]
gdb   = [r[2] for r in rows]
phase = [r[3] for r in rows]
clip  = [r[7] for r in rows]
peak  = max(gdb)

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
ax1.semilogx(f, gdb, "o-")
ax1.axhline(peak - 3, ls="--", c="r", label="-3 dB")
for fi, gi, ci in zip(f, gdb, clip):
    if ci:
        ax1.annotate("clip", (fi, gi), color="r")
ax1.set_ylabel("Gain (dB)")
ax1.legend()
ax1.grid(True, which="both")

ax2.semilogx(f, phase, "o-")
ax2.set_ylabel("Phase (deg)")
ax2.set_xlabel("Frequency (Hz)")
ax2.grid(True, which="both")

fig.suptitle("ELEC2104 Stage II - measured Bode plot")
fig.tight_layout()
fig.savefig("stage2_bode.png", dpi=150)
plt.show()