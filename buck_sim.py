# buck_sim.py
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# --------------------------
# Parameters
# --------------------------
Vin = 12.0        # Input voltage [V]
Vref = 5.0        # Reference output voltage [V]
L = 47e-6         # Inductor [H]
C = 100e-6        # Capacitor [F]
Rload = 10.0      # Load resistance [ohm]
Kp, Ki = 0.05, 500  # PI controller gains

dt = 1e-6         # Simulation step [s]
tmax = 2e-3       # Total time [s]
N = int(tmax/dt)  # Steps

# --------------------------
# State variables
# --------------------------
Vout = 0.0
IL = 0.0
duty = 0.5
integral = 0.0

# Traces
time_trace, vout_trace, il_trace, duty_trace, fault_trace = [], [], [], [], []

# --------------------------
# Simulation loop
# --------------------------
for i in range(N):
    t = i*dt

    # Controller (PI)
    error = Vref - Vout
    integral += error * dt
    duty = Kp * error + Ki * integral
    duty = max(0.0, min(1.0, duty))  # clamp between 0 and 1

    # Buck converter averaged model
    dIL = (Vin*duty - Vout) / L * dt
    IL += dIL
    dVout = (IL - Vout/Rload) / C * dt
    Vout += dVout

    # Fault check (e.g., overvoltage)
    fault = 1 if Vout > 1.2*Vref else 0

    # Log
    time_trace.append(t)
    vout_trace.append(Vout)
    il_trace.append(IL)
    duty_trace.append(duty)
    fault_trace.append(fault)

# --------------------------
# Save results
# --------------------------
df = pd.DataFrame({
    "time_s": time_trace,
    "Vout_V": vout_trace,
    "IL_A": il_trace,
    "Duty": duty_trace,
    "Fault": fault_trace
})
df.to_csv("buck_sim_traces.csv", index=False)

# --------------------------
# Plots
# --------------------------
plt.figure()
plt.plot(time_trace, vout_trace)
plt.xlabel("Time [s]"); plt.ylabel("Vout [V]")
plt.title("Buck Converter Output Voltage")
plt.grid(True)
plt.savefig("vout_plot.png")

plt.figure()
plt.plot(time_trace, il_trace)
plt.xlabel("Time [s]"); plt.ylabel("Inductor Current [A]")
plt.title("Inductor Current")
plt.grid(True)
plt.savefig("iload_plot.png")

plt.figure()
plt.plot(time_trace, duty_trace)
plt.xlabel("Time [s]"); plt.ylabel("Duty Cycle")
plt.title("Controller Duty Cycle")
plt.grid(True)
plt.savefig("duty_plot.png")

plt.figure()
plt.plot(time_trace, fault_trace)
plt.xlabel("Time [s]"); plt.ylabel("Fault Flag")
plt.title("Fault Detection")
plt.grid(True)
plt.savefig("fault_plot.png")

print("Simulation complete. Results saved: buck_sim_traces.csv + plots.")
