#!/usr/bin/env python3
# bms_2s2p_can.py
# 2S2P BMS simulation with SOC, faults, balancing, shutdown, and CAN logging
# Save as bms_2s2p_can.py and run with: python bms_2s2p_can.py

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

# --------------------------
# Simulation parameters
# --------------------------
dt = 1.0               # timestep [s]
t_end = 600            # total simulation time [s] (10 minutes)
t = np.arange(0, t_end+dt, dt)
N = len(t)

# Topology
cells_per_group = 2
groups = 2
num_cells = groups * cells_per_group  # 4 cells total

# Cell parameters
Q_nom_ah = 2.5            # Ah
Q_nom = Q_nom_ah * 3600.0 # Coulombs
R_internal = 0.05         # ohm
OCV_full = 4.2
OCV_empty = 3.0

# Initial SOC with slight imbalance
SOC = np.array([0.95, 0.93, 0.94, 0.92])  # per-cell initial SOC [0..1]

# OCV mapping (simple linear)
def ocv_from_soc(s):
    return OCV_empty + (OCV_full - OCV_empty) * s

# Current profile: nominal discharge, short spike, persistent overcurrent window
I_pack_nom = 1.0
I_pack_fault = 8.0
current_profile = np.ones(N) * I_pack_nom
for idx, time in enumerate(t):
    if 118 <= time < 122:
        current_profile[idx] = 5.0
    if 240 <= time < 360:
        current_profile[idx] = I_pack_fault

# Temperature and balancing config
T_ambient = 25.0
cell_temp = np.ones(num_cells) * T_ambient
balance_threshold_soc = 0.05
bleed_resistance = 50.0
balance_enabled = True

# Fault thresholds
V_cell_ov = 4.15
V_cell_uv = 2.9
T_over = 60.0
T_under = -20.0
I_over = 6.0

# Persistence
persistence_time = 5.0
persistence_steps = int(persistence_time / dt)

# Traces
cell_voltage = np.zeros((N, num_cells))
group_voltage = np.zeros((N, groups))
pack_voltage = np.zeros(N)
cell_soc = np.zeros((N, num_cells))
cell_temp_trace = np.zeros((N, num_cells))
pack_current_trace = current_profile.copy()
fault_flags = np.zeros((N, 6), dtype=int)  # OV, UV, OT, UT, IC, IMB

# CAN log
can_log = []

# Persistence counters
ov_pcnt = np.zeros(num_cells, dtype=int)
uv_pcnt = np.zeros(num_cells, dtype=int)
temp_pcnt = np.zeros(num_cells, dtype=int)
ic_pcnt = 0
imbalance_pcnt = 0

# Simple helper functions
def compute_cell_voltage(soc, i_cell):
    return ocv_from_soc(soc) - i_cell * R_internal

def split_current_among_parallel(group_cells, pack_current):
    socs = SOC[group_cells].clip(min=0.01)
    weights = socs / np.sum(socs)
    return weights * pack_current

# init
for c in range(num_cells):
    cell_voltage[0,c] = ocv_from_soc(SOC[c])
    cell_soc[0,c] = SOC[c]
    cell_temp_trace[0,c] = cell_temp[c]

shutdown = False
cooling_on = False
balance_state = np.zeros(num_cells, dtype=int)
fault_latched = np.zeros(6, dtype=int)

# simulation loop
for k in range(0, N-1):
    time = t[k]
    I_pack = current_profile[k]
    if shutdown:
        I_pack = 0.0

    # split currents for each group (parallel share)
    group_currents = np.zeros((groups, cells_per_group))
    for g in range(groups):
        idxs = list(range(g*cells_per_group, (g+1)*cells_per_group))
        split = split_current_among_parallel(idxs, I_pack)
        for i_local, ci in enumerate(idxs):
            group_currents[g, i_local] = split[i_local]

    # coulomb counting
    for c in range(num_cells):
        g = c // cells_per_group
        local = c % cells_per_group
        i_cell = group_currents[g, local]
        dQ = i_cell * dt
        SOC[c] -= dQ / Q_nom
        SOC[c] = np.clip(SOC[c], 0.0, 1.0)

    # temperature rise (simple)
    for c in range(num_cells):
        g = c // cells_per_group
        local = c % cells_per_group
        i_cell = group_currents[g, local]
        deltaT = (i_cell**2) * R_internal * 0.01
        cell_temp[c] += deltaT
        if cooling_on:
            cell_temp[c] -= 0.1

    # compute voltages
    for c in range(num_cells):
        g = c // cells_per_group
        local = c % cells_per_group
        i_cell = group_currents[g, local]
        v = compute_cell_voltage(SOC[c], i_cell)
        cell_voltage[k+1, c] = v
        cell_soc[k+1, c] = SOC[c]
        cell_temp_trace[k+1, c] = cell_temp[c]

    for g in range(groups):
        idxs = list(range(g*cells_per_group, (g+1)*cells_per_group))
        group_voltage[k+1, g] = np.mean(cell_voltage[k+1, idxs])
    pack_voltage[k+1] = np.sum(group_voltage[k+1, :])

    # fault detection with persistence
    ov_flags = (cell_voltage[k+1,:] > V_cell_ov).astype(int)
    uv_flags = (cell_voltage[k+1,:] < V_cell_uv).astype(int)
    ot_flags = (cell_temp > T_over).astype(int)
    ut_flags = (cell_temp < T_under).astype(int)
    ic_flag = abs(I_pack) > I_over

    for c in range(num_cells):
        ov_pcnt[c] = ov_pcnt[c] + 1 if ov_flags[c] else 0
        uv_pcnt[c] = uv_pcnt[c] + 1 if uv_flags[c] else 0
        temp_pcnt[c] = temp_pcnt[c] + 1 if ot_flags[c] else 0

    ic_pcnt = ic_pcnt + 1 if ic_flag else 0

    imbalance = False
    for g in range(groups):
        idxs = list(range(g*cells_per_group, (g+1)*cells_per_group))
        if np.max(SOC[idxs]) - np.min(SOC[idxs]) > balance_threshold_soc:
            imbalance = True
    imbalance_pcnt = imbalance_pcnt + 1 if imbalance else 0

    ov_persistent = (ov_pcnt >= persistence_steps)
    uv_persistent = (uv_pcnt >= persistence_steps)
    temp_persistent = (temp_pcnt >= persistence_steps)
    ic_persistent = (ic_pcnt >= persistence_steps)
    imbalance_persistent = (imbalance_pcnt >= persistence_steps)

    flag_OV = int(np.any(ov_persistent))
    flag_UV = int(np.any(uv_persistent))
    flag_OT = int(np.any(temp_persistent))
    flag_UT = int(np.any(ut_flags))
    flag_IC = int(ic_persistent)
    flag_IMB = int(imbalance_persistent)

    if flag_OV or flag_UV or flag_OT or flag_UT or flag_IC or flag_IMB:
        fault_latched = np.array([flag_OV or fault_latched[0],
                                  flag_UV or fault_latched[1],
                                  flag_OT or fault_latched[2],
                                  flag_UT or fault_latched[3],
                                  flag_IC or fault_latched[4],
                                  flag_IMB or fault_latched[5]], dtype=int)

    # actions
    if flag_OT:
        cooling_on = True
    if imbalance_persistent and balance_enabled:
        for g in range(groups):
            idxs = list(range(g*cells_per_group, (g+1)*cells_per_group))
            idx_max = idxs[np.argmax(SOC[idxs])]
            balance_state[idx_max] = 1
    else:
        balance_state[:] = 0

    if flag_OT or flag_OV or flag_IC:
        shutdown = True

    # balancing bleed effect
    for c in range(num_cells):
        if balance_state[c] == 1:
            bleed_I = cell_voltage[k+1,c] / bleed_resistance
            SOC[c] = max(0.0, SOC[c] - bleed_I * dt / Q_nom)

    fault_flags[k+1,:] = np.array([flag_OV, flag_UV, int(flag_OT), int(flag_UT), flag_IC, flag_IMB], dtype=int)

    # CAN messages every 5 seconds
    if (k % 5) == 0:
        can_log.append({
            'time_s': time, 'id': 0x100, 'name':'PACK_STATUS',
            'pack_voltage_V': float(pack_voltage[k+1]), 'pack_current_A': float(I_pack),
            'cooling_on': int(cooling_on), 'shutdown': int(shutdown)
        })
        can_log.append({
            'time_s': time, 'id': 0x200, 'name':'CELL_VOLTAGES',
            'v0': float(cell_voltage[k+1,0]), 'v1': float(cell_voltage[k+1,1]),
            'v2': float(cell_voltage[k+1,2]), 'v3': float(cell_voltage[k+1,3])
        })
        can_log.append({
            'time_s': time, 'id': 0x300, 'name':'CELL_SOCS',
            's0': float(SOC[0]), 's1': float(SOC[1]), 's2': float(SOC[2]), 's3': float(SOC[3])
        })
        can_log.append({
            'time_s': time, 'id': 0x400, 'name':'FAULT_SUMMARY',
            'OV': int(flag_OV), 'UV': int(flag_UV), 'OT': int(flag_OT),
            'UT': int(flag_UT), 'IC': int(flag_IC), 'IMB': int(flag_IMB)
        })

# final fill
pack_voltage[-1] = np.sum(np.mean(cell_voltage[-1].reshape(groups, cells_per_group), axis=1))
cell_soc[-1,:] = SOC
cell_temp_trace[-1,:] = cell_temp

# save outputs
os.makedirs("bms_outputs/plots", exist_ok=True)
df = pd.DataFrame({
    'time_s': t, 'pack_V': pack_voltage, 'I_pack_A': pack_current_trace,
    'cell_v0': cell_voltage[:,0], 'cell_v1': cell_voltage[:,1],
    'cell_v2': cell_voltage[:,2], 'cell_v3': cell_voltage[:,3],
    'soc0': cell_soc[:,0], 'soc1': cell_soc[:,1],
    'soc2': cell_soc[:,2], 'soc3': cell_soc[:,3],
    'temp0': cell_temp_trace[:,0], 'temp1': cell_temp_trace[:,1],
    'temp2': cell_temp_trace[:,2], 'temp3': cell_temp_trace[:,3],
    'ov_flag': fault_flags[:,0], 'uv_flag': fault_flags[:,1],
    'ot_flag': fault_flags[:,2], 'ut_flag': fault_flags[:,3],
    'ic_flag': fault_flags[:,4], 'imb_flag': fault_flags[:,5]
})
df.to_csv("bms_outputs/bms_traces.csv", index=False)
pd.DataFrame(can_log).to_csv("bms_outputs/can_log.csv", index=False)

# plots
plt.figure(); plt.plot(t, pack_voltage); plt.xlabel('s'); plt.ylabel('V'); plt.title('Pack Voltage'); plt.grid(True); plt.savefig("bms_outputs/plots/pack_voltage.png", bbox_inches='tight')
plt.figure(); plt.plot(t, cell_voltage[:,0], label='v0'); plt.plot(t, cell_voltage[:,1], label='v1'); plt.plot(t, cell_voltage[:,2], label='v2'); plt.plot(t, cell_voltage[:,3], label='v3'); plt.legend(); plt.title('Cell Voltages'); plt.grid(True); plt.savefig("bms_outputs/plots/cell_voltages.png", bbox_inches='tight')
plt.figure(); plt.plot(t, cell_soc[:,0], label='soc0'); plt.plot(t, cell_soc[:,1], label='soc1'); plt.plot(t, cell_soc[:,2], label='soc2'); plt.plot(t, cell_soc[:,3], label='soc3'); plt.legend(); plt.title('SOC'); plt.grid(True); plt.savefig("bms_outputs/plots/socs.png", bbox_inches='tight')
plt.figure(); plt.plot(t, cell_temp_trace[:,0], label='T0'); plt.plot(t, cell_temp_trace[:,1], label='T1'); plt.plot(t, cell_temp_trace[:,2], label='T2'); plt.plot(t, cell_temp_trace[:,3], label='T3'); plt.legend(); plt.title('Temps'); plt.grid(True); plt.savefig("bms_outputs/plots/temps.png", bbox_inches='tight')

# CAN rate plot
can_df = pd.DataFrame(can_log)
can_times = can_df['time_s'].values if not can_df.empty else np.array([])
window = 60
centers = []
rates = []
for start in range(0, int(t_end), 10):
    cnt = np.sum((can_times >= start) & (can_times < start + window))
    centers.append(start + window/2)
    rates.append(cnt)
plt.figure(); plt.plot(centers, rates, marker='o'); plt.title('CAN msgs per 60s window'); plt.grid(True); plt.savefig("bms_outputs/plots/can_rate.png", bbox_inches='tight')

print("Saved outputs to folder: bms_outputs/")
print("Sample CAN log (first rows):")
print(can_df.head(8).to_string(index=False))
