import numpy as np
import matplotlib.pyplot as plt

# Measured raw counts from pot
angle = np.array([0, 45, 90, 135, 180, 225, 270, 280])
adc = np.array([5, 162, 328, 493, 653, 822, 991, 1020])

# Straight line fit
slope, offset = np.polyfit(angle, adc, 1)
predictions = slope * angle + offset
r2 = 1 - np.sum((adc - predictions) ** 2) / np.sum((adc - adc.mean()) ** 2)

# Conversion error
error = (adc - offset) / slope - angle
print(f"ADC = {slope:.3f} * angle + {offset:.1f} \nR^2 = {r2:.5f}")
print(f"Max error: {np.max(np.abs(error)):.2f} deg, RMS: {np.sqrt(np.mean(error**2)):.2f} deg")
print(f"Measured electrical travel: {(1023 - offset) / slope:.1f} deg (datasheet 280)")
print("errors:", np.round(error, 2))

# Create transfer function plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 7), sharex=True, gridspec_kw={"height_ratios": [3, 2]})
ax1.plot(angle, adc, "o", label="Measured")
ax1.plot(angle, predictions, "-", label=f"Fit: ADC = {slope:.3f}·θ + {offset:.1f}  (R² = {r2:.4f})")
ax1.set_ylabel("ADC counts (0–1023)")
ax1.set_title("Potentiometer transfer function (Bourns 3386)")
ax1.legend(); ax1.grid(alpha=0.3)
ax2.axhline(0, color="black", lw=0.8)
ax2.plot(angle, error, "o-", color="C1",
         label=f"Max {np.max(np.abs(error)):.2f}°, RMS {np.sqrt(np.mean(error**2)):.2f}°")
ax2.set_xlabel("Knob angle (degrees)")
ax2.set_ylabel("Conversion error (degrees)")
ax2.legend(); ax2.grid(alpha=0.3)
fig.tight_layout()
plt.savefig("pot_tf_plot.png", dpi=150)