# import matplotlib.pyplot as plt
# import numpy as np
# import seaborn as sns
# from matplotlib.ticker import LogLocator, FuncFormatter, NullFormatter
# from matplotlib.lines import Line2D
# from matplotlib.patches import Patch

# # --- Define common markers ---
# marker_dji_drones = '^'       # Triangle up for DJI Drones
# marker_large_vtol = 's'       # Square for Helicopters and other large VTOLs
# marker_crazyflie_custom = '+' # Keep original for these
# marker_floaty = 'o'

# # --- Data for the plot (Mass in Grams) ---
# data = {
#     # Drones
#     'Floaty': {'mass_g': 340, 'power_still_W': None, 'power_updraft_W': 3.4, 'color': 'green', 'marker_override': marker_floaty}, # Floaty marker
#     'Crazyflie 2.1': {'mass_g': 27, 'power_still_W': 8.0, 'power_updraft_W': 6.6, 'color': 'dodgerblue', 'marker_override': marker_crazyflie_custom},
#     'Custom Quadcopter': {'mass_g': 950, 'power_still_W': 137.75, 'power_updraft_W': 64.6, 'color': 'firebrick', 'marker_override': marker_crazyflie_custom},

#     # DJI Drones - will use common marker_dji_drones
#     'DJI Mini 4 Pro': {'mass_g': 249, 'power_still_W': 33.4, 'power_updraft_W': None, 'color': 'purple', 'type': 'dji'},
#     'DJI Air 3': {'mass_g': 720, 'power_still_W': 81.6, 'power_updraft_W': None, 'color': 'orange', 'type': 'dji'},
#     'DJI Mavic 3 Pro': {'mass_g': 958, 'power_still_W': 107.4, 'power_updraft_W': None, 'color': 'teal', 'type': 'dji'},
#     'DJI Avata': {'mass_g': 410, 'power_still_W': 119, 'power_updraft_W': None, 'color': 'magenta', 'type': 'dji'},

#     # Helicopters & eVTOLs (Lilium Jet REMOVED) - will use common marker_large_vtol
#     'Robinson R22': {'mass_g': 622 * 1000, 'power_still_W': 77.5 * 1000, 'power_updraft_W': None, 'color': '#FFC300', 'type': 'helicopter'},
#     'Bell 206B JetRanger': {'mass_g': 1451 * 1000, 'power_still_W': 165 * 1000, 'power_updraft_W': None, 'color': '#FF5733', 'type': 'helicopter'},
#     'Airbus H125': {'mass_g': 2250 * 1000, 'power_still_W': 415 * 1000, 'power_updraft_W': None, 'color': '#C70039', 'type': 'helicopter'},
#     'UH-60M Black Hawk': {'mass_g': 9979 * 1000, 'power_still_W': 1700 * 1000, 'power_updraft_W': None, 'color': '#900C3F', 'type': 'helicopter'},
#     'Joby S4 (eVTOL)': {'mass_g': 2177 * 1000, 'power_still_W': 350 * 1000, 'power_updraft_W': None, 'color': '#2ECC71', 'type': 'helicopter'}, # Grouping Joby with large VTOLs marker-wise
#     # 'Lilium Jet (eVTOL)': {'mass_g': 3175 * 1000, 'power_still_W': 1750 * 1000, 'power_updraft_W': None, 'color': '#3498DB', 'marker_override': '8'}, # REMOVED
# }

# # --- Plotting ---
# sns.set_theme(style="whitegrid", rc={"grid.linestyle": ":", "grid.alpha": 0.7})
# plt.figure(figsize=(10, 7))

# title_fontsize = 14
# axis_label_fontsize = 12
# tick_label_fontsize = 10
# legend_fontsize = 8.5 # Slightly increased legend font size

# scatter_marker_size_default = 110 # General size for common markers
# scatter_marker_size_plus_star = 130 # For '+' and '*' markers
# legend_marker_size = 7
# line_width_main = 1.8
# line_width_secondary = 1.5

# marker_updraft_default = '*' # Default updraft marker

# # --- Lines of Constant Specific Power ---
# all_masses_current = [props['mass_g'] for props in data.values()]
# min_mass_overall = min(all_masses_current) if all_masses_current else 10
# max_mass_overall = max(all_masses_current) if all_masses_current else 1e7

# mass_range_g = np.logspace(np.log10(min_mass_overall*0.5 if min_mass_overall > 0 else 10),
#                            np.log10(max_mass_overall*1.5 if max_mass_overall > 0 else 1e7), 100)

# specific_power_floaty_W_g = 0.01; power_floaty_line = specific_power_floaty_W_g * mass_range_g
# specific_power_ref_W_g = 0.1; power_ref_line = specific_power_ref_W_g * mass_range_g
# specific_power_high_W_g = 0.3; power_high_line = specific_power_high_W_g * mass_range_g
# # Removed 600 W/kg line as Lilium is removed
# # specific_power_vhigh_W_g = 0.6; power_vhigh_line = specific_power_vhigh_W_g * mass_range_g

# fill_color = 'lightgray'; fill_alpha = 0.3
# plt.fill_between(mass_range_g, power_ref_line, power_high_line, color=fill_color, alpha=fill_alpha, zorder=1) # Label will be in custom legend

# plt.plot(mass_range_g, power_floaty_line, 'k--', alpha=0.9, linewidth=line_width_main, zorder=3)
# plt.plot(mass_range_g, power_ref_line, 'dimgrey', linestyle=':', alpha=0.8, linewidth=line_width_secondary, zorder=3)
# plt.plot(mass_range_g, power_high_line, 'slategray', linestyle=':', alpha=0.8, linewidth=line_width_secondary, zorder=3)
# # plt.plot(mass_range_g, power_vhigh_line, 'darkred', linestyle=':', alpha=0.7, linewidth=line_width_secondary, zorder=3)


# # Plotting data points
# for name, props in data.items():
#     vehicle_type = props.get('type', None)
#     marker_override = props.get('marker_override', None)

#     current_marker_still = marker_crazyflie_custom # Default
#     if marker_override:
#         current_marker_still = marker_override
#     elif vehicle_type == 'dji':
#         current_marker_still = marker_dji_drones
#     elif vehicle_type == 'helicopter': # Includes Joby S4 as per grouping
#         current_marker_still = marker_large_vtol

#     current_marker_updraft = marker_updraft_default
#     if name == 'Floaty': # Floaty has specific updraft handling already via marker_override
#         pass
#     elif props['power_updraft_W'] is not None and vehicle_type == 'dji': # Example if DJI had updraft with same marker
#          current_marker_updraft = marker_dji_drones
#     elif props['power_updraft_W'] is not None and vehicle_type == 'helicopter':
#          current_marker_updraft = marker_large_vtol


#     marker_size_still = scatter_marker_size_plus_star if current_marker_still in ['+', '*'] else scatter_marker_size_default
#     marker_size_updraft = scatter_marker_size_plus_star if current_marker_updraft == '*' else scatter_marker_size_default


#     if props['power_still_W'] is not None:
#         plt.scatter(props['mass_g'], props['power_still_W'], s=marker_size_still,
#                     color=props['color'], marker=current_marker_still, alpha=0.9, ec=props['color'], zorder=5,
#                     linewidths=1.2 if current_marker_still in ['+', '*'] else 0.5, label=name) # Keep individual labels for now
#     if props['power_updraft_W'] is not None:
#         # Floaty's updraft point uses its specific marker_override
#         updraft_marker_to_use = current_marker_still if name == 'Floaty' else current_marker_updraft

#         plt.scatter(props['mass_g'], props['power_updraft_W'], s=marker_size_updraft,
#                     color=props['color'], marker=updraft_marker_to_use, alpha=1.0, ec=props['color'], zorder=5,
#                     linewidths=1.0 if updraft_marker_to_use == '*' else 0.5, label=name + " (Updraft)")


# plt.xscale('log'); plt.yscale('log')
# plt.xlabel('Mass (g)', fontsize=axis_label_fontsize, fontweight='bold')
# plt.ylabel('Power Consumption (W)', fontsize=axis_label_fontsize, fontweight='bold')
# plt.title('Power Consumption vs. Mass: VTOL Aircraft', fontsize=title_fontsize, fontweight='bold')

# def log_formatter_kilograms_kilowatts(x, pos):
#     if x >= 1e6: return f'{x/1e6:.1f}M'
#     if x >= 1e3: return f'{x/1e3:.0f}k'
#     if x >=1: return f'{x:.0f}'
#     return f'{x:.1f}'

# ax = plt.gca()
# ax.xaxis.set_major_formatter(FuncFormatter(log_formatter_kilograms_kilowatts))
# ax.yaxis.set_major_formatter(FuncFormatter(log_formatter_kilograms_kilowatts))
# ax.xaxis.set_major_locator(LogLocator(base=10.0, numticks=12))
# ax.yaxis.set_major_locator(LogLocator(base=10.0, numticks=12))
# ax.xaxis.set_minor_formatter(NullFormatter())
# ax.yaxis.set_minor_formatter(NullFormatter())
# plt.xticks(fontsize=tick_label_fontsize, rotation=45, ha="right")
# plt.yticks(fontsize=tick_label_fontsize)

# all_powers = [p for d_val in data.values() for p in [d_val['power_still_W'], d_val['power_updraft_W']] if p is not None]
# if all_powers: plt.ylim(min(all_powers) * 0.1, max(all_powers) * 10)
# if all_masses_current: plt.xlim(min_mass_overall * 0.1, max_mass_overall * 10)


# # --- Custom Legend ---
# # Get handles/labels from scatter (individual points)
# plotted_handles, plotted_labels = ax.get_legend_handles_labels()

# # Create unique entries for marker types if you want to group them in legend
# # This is a bit more complex if colors also vary within a marker type
# # For now, let's keep individual entries, but common markers will be visually apparent.
# # If you want a single legend entry "DJI Drones" with the triangle, you'd need to plot them differently or manually craft that legend entry.

# # Legend elements for lines and patch
# line_patch_handles = [
#     Line2D([0], [0], color='k', lw=line_width_main, linestyle='--', label='10 W/kg'),
#     Line2D([0], [0], color='dimgrey', lw=line_width_secondary, linestyle=':', label='100 W/kg'),
#     Line2D([0], [0], color='slategray', lw=line_width_secondary, linestyle=':', label='300 W/kg'),
#     # Removed 600 W/kg line
#     Patch(facecolor=fill_color, edgecolor='darkgrey', alpha=fill_alpha, label='100-300 W/kg Range')
# ]
# line_patch_labels = [h.get_label() for h in line_patch_handles]

# # Combine (ensure no duplicate labels from lines if they were auto-added, though they weren't here)
# combined_handles = plotted_handles + line_patch_handles
# combined_labels = plotted_labels + line_patch_labels

# # Filter out duplicate labels for legend, keeping first occurrence
# # This is useful if scatter points and lines might accidentally share labels
# unique_labels_map = {}
# final_handles = []
# final_labels = []
# for handle, label in zip(combined_handles, combined_labels):
#     if label not in unique_labels_map:
#         unique_labels_map[label] = handle
#         final_handles.append(handle)
#         final_labels.append(label)


# ax.legend(handles=final_handles, labels=final_labels, fontsize=legend_fontsize,
#           loc='center left', bbox_to_anchor=(1.02, 0.5),
#           frameon=True, facecolor='white', framealpha=0.85, ncol=1)

# ax.grid(True, which="major", ls="-", alpha=0.4, color='lightgray', zorder=0, linewidth=0.8)
# plt.subplots_adjust(left=0.1, right=0.65, top=0.92, bottom=0.15) # Adjusted right for potentially wider legend items

# pdf_filename = "/home/gelmkaiel/Floaty/Nature Machine Intelligence/Figures/power_vs_mass_consolidated_markers.pdf"
# plt.savefig(pdf_filename, format='pdf', dpi=300)
# print(f"Figure saved as {pdf_filename}")
# plt.show()



import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib.ticker import LogLocator, FuncFormatter, NullFormatter
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

# --- Define common markers ---
marker_dji_drones = '^'
marker_large_vtol = 's'
marker_crazyflie_custom = '+'
marker_floaty = 'o'

# --- Data for the plot (Mass originally in Grams) ---
data = {
    'Floaty': {'mass_g': 340, 'power_still_W': None, 'power_updraft_W': 3.4, 'color': 'green', 'marker_override': marker_floaty},
    'Crazyflie 2.1': {'mass_g': 27, 'power_still_W': 8.0, 'power_updraft_W': 6.6, 'color': 'dodgerblue', 'marker_override': marker_crazyflie_custom},
    'Custom Quad': {'mass_g': 950, 'power_still_W': 137.75, 'power_updraft_W': 64.6, 'color': 'firebrick', 'marker_override': marker_crazyflie_custom},
    'DJI Mini 4 Pro': {'mass_g': 249, 'power_still_W': 33.4, 'power_updraft_W': None, 'color': 'purple', 'type': 'dji'},
    'DJI Air 3': {'mass_g': 720, 'power_still_W': 81.6, 'power_updraft_W': None, 'color': 'orange', 'type': 'dji'},
    # 'DJI Mavic 3 Pro': {'mass_g': 958, 'power_still_W': 107.4, 'power_updraft_W': None, 'color': 'teal', 'type': 'dji'},
    'DJI Avata': {'mass_g': 410, 'power_still_W': 119, 'power_updraft_W': None, 'color': 'magenta', 'type': 'dji'},
    'Robinson R22': {'mass_g': 622 * 1000, 'power_still_W': 77.5 * 1000, 'power_updraft_W': None, 'color': '#FFC300', 'type': 'helicopter'},
    'Bell 206B': {'mass_g': 1451 * 1000, 'power_still_W': 165 * 1000, 'power_updraft_W': None, 'color': '#FF5733', 'type': 'helicopter'},
    'Airbus H125': {'mass_g': 2250 * 1000, 'power_still_W': 415 * 1000, 'power_updraft_W': None, 'color': '#C70039', 'type': 'helicopter'},
    'UH-60M Black Hawk': {'mass_g': 9979 * 1000, 'power_still_W': 1700 * 1000, 'power_updraft_W': None, 'color': '#900C3F', 'type': 'helicopter'},
    'Joby S4 (eVTOL)': {'mass_g': 2177 * 1000, 'power_still_W': 350 * 1000, 'power_updraft_W': None, 'color': '#2ECC71', 'type': 'helicopter'},
}

# Convert mass to kg for all vehicles
for name in data:
    data[name]['mass_kg'] = data[name]['mass_g'] / 1000.0

# --- Plotting ---
sns.set_theme(style="whitegrid", rc={"grid.linestyle": ":", "grid.alpha": 0.7})

# << CHANGE FIGURE SIZE FOR 1x4 ASPECT RATIO (approx) >>
# Example: width=16, height=4 or width=12, height=3
fig_width = 16
fig_height = 7.5 # Slightly more height for legend below
plt.figure(figsize=(fig_width, fig_height))

title_fontsize = 14
axis_label_fontsize = 12
tick_label_fontsize = 10
legend_fontsize = 9 # Adjusted for legend below

scatter_marker_size_default = 90
scatter_marker_size_plus_star = 110
legend_marker_size = 6 # For legend entries
line_width_main = 1.8
line_width_secondary = 1.5

marker_updraft_default = '*'

# --- Lines of Constant Specific Power (using mass in KG) ---
all_masses_current_kg = [props['mass_kg'] for props in data.values()]
min_mass_overall_kg = min(all_masses_current_kg) if all_masses_current_kg else 0.01
max_mass_overall_kg = max(all_masses_current_kg) if all_masses_current_kg else 10000 # Max mass in kg (e.g., 10 tonnes)

mass_range_kg = np.logspace(np.log10(min_mass_overall_kg * 0.5 if min_mass_overall_kg > 0 else 0.01),
                           np.log10(max_mass_overall_kg * 1.5 if max_mass_overall_kg > 0 else 10000), 100)

# Specific power values are now directly in W/kg
sp_10_W_kg = 10
power_line_10 = sp_10_W_kg * mass_range_kg

sp_100_W_kg = 100
power_line_100 = sp_100_W_kg * mass_range_kg

sp_300_W_kg = 300
power_line_300 = sp_300_W_kg * mass_range_kg

fill_color = 'lightgray'; fill_alpha = 0.3
# Use mass_range_kg for fill_between
plt.fill_between(mass_range_kg, power_line_100, power_line_300, color=fill_color, alpha=fill_alpha, zorder=1)

plt.plot(mass_range_kg, power_line_10, 'k--', alpha=0.9, linewidth=line_width_main, zorder=3)
plt.plot(mass_range_kg, power_line_100, 'dimgrey', linestyle=':', alpha=0.8, linewidth=line_width_secondary, zorder=3)
plt.plot(mass_range_kg, power_line_300, 'slategray', linestyle=':', alpha=0.8, linewidth=line_width_secondary, zorder=3)

# Plotting data points (using mass_kg)
for name, props in data.items():
    vehicle_type = props.get('type', None)
    marker_override = props.get('marker_override', None)
    mass_kg_val = props['mass_kg'] # Use mass in kg

    current_marker_still = marker_crazyflie_custom
    if marker_override: current_marker_still = marker_override
    elif vehicle_type == 'dji': current_marker_still = marker_dji_drones
    elif vehicle_type == 'helicopter': current_marker_still = marker_large_vtol

    current_marker_updraft = marker_updraft_default

    marker_size_still = scatter_marker_size_plus_star if current_marker_still in ['+', '*'] else scatter_marker_size_default
    marker_size_updraft = scatter_marker_size_plus_star if current_marker_updraft == '*' else scatter_marker_size_default

    if props['power_still_W'] is not None:
        plt.scatter(mass_kg_val, props['power_still_W'], s=marker_size_still,
                    color=props['color'], marker=current_marker_still, alpha=0.9, ec=props['color'], zorder=5,
                    linewidths=1.2 if current_marker_still in ['+', '*'] else 0.5, label=name)
    if props['power_updraft_W'] is not None:
        updraft_marker_to_use = current_marker_still if name == 'Floaty' else current_marker_updraft
        plt.scatter(mass_kg_val, props['power_updraft_W'], s=marker_size_updraft,
                    color=props['color'], marker=updraft_marker_to_use, alpha=1.0, ec=props['color'], zorder=5,
                    linewidths=1.0 if updraft_marker_to_use == '*' else 0.5, label=name + " (Updraft)")

plt.xscale('log'); plt.yscale('log')
# << CHANGE X-AXIS LABEL >>
plt.xlabel('Mass (kg)', fontsize=axis_label_fontsize, fontweight='bold')
plt.ylabel('Power Consumption (W)', fontsize=axis_label_fontsize, fontweight='bold')
plt.title('Power Consumption vs. Mass: VTOL Aircraft', fontsize=title_fontsize, fontweight='bold')

def log_formatter_value_unit(x, pos): # Renamed for clarity
    if x >= 1e6: return f'{x/1e6:.1f}M' # M for Mega (e.g., MW or Mkg - though Mkg is unusual, it's consistent)
    if x >= 1e3: return f'{x/1e3:.0f}k' # k for kilo (e.g., kW or kg)
    if x >=1: return f'{x:.0f}'
    if x < 1 and x > 0: # For masses < 1kg
        if x > 0.01: return f'{x:.2f}'
        return f'{x:.3f}'
    return f'{x:.1f}' # Default for other small numbers or if logic missed something

ax = plt.gca()
ax.xaxis.set_major_formatter(FuncFormatter(log_formatter_value_unit))
ax.yaxis.set_major_formatter(FuncFormatter(log_formatter_value_unit)) # y-axis (Power) also uses this
ax.xaxis.set_major_locator(LogLocator(base=10.0, numticks=15)) # More ticks might be ok on wide plot
ax.yaxis.set_major_locator(LogLocator(base=10.0, numticks=10))
ax.xaxis.set_minor_formatter(NullFormatter())
ax.yaxis.set_minor_formatter(NullFormatter())
plt.xticks(fontsize=tick_label_fontsize, rotation=30, ha="right") # Adjusted rotation for wide plot
plt.yticks(fontsize=tick_label_fontsize)

# Automatic limits based on all data (mass in kg)
all_powers = [p for d_val in data.values() for p in [d_val['power_still_W'], d_val['power_updraft_W']] if p is not None]
if all_powers: plt.ylim(min(all_powers) * 0.1, max(all_powers) * 10)
if all_masses_current_kg: plt.xlim(min_mass_overall_kg * 0.1, max_mass_overall_kg * 10)


# --- Custom Legend (placed below the plot) ---
plotted_handles, plotted_labels = ax.get_legend_handles_labels()
line_patch_handles = [
    Line2D([0], [0], color='k', lw=line_width_main, linestyle='--', label='10 W/kg'),
    Line2D([0], [0], color='dimgrey', lw=line_width_secondary, linestyle=':', label='100 W/kg'),
    Line2D([0], [0], color='slategray', lw=line_width_secondary, linestyle=':', label='300 W/kg'),
    Patch(facecolor=fill_color, edgecolor='darkgrey', alpha=fill_alpha, label='100-300 W/kg Range')
]
line_patch_labels = [h.get_label() for h in line_patch_handles]

combined_handles = plotted_handles + line_patch_handles
combined_labels = plotted_labels + line_patch_labels

unique_labels_map = {}
final_handles = []
final_labels = []
for handle, label in zip(combined_handles, combined_labels):
    if label not in unique_labels_map:
        unique_labels_map[label] = handle
        final_handles.append(handle)
        final_labels.append(label)

# << ADJUST LEGEND FOR HORIZONTAL LAYOUT BELOW PLOT >>
num_legend_cols = 5 # Adjust based on number of items and figure width
ax.legend(handles=final_handles, labels=final_labels, fontsize=legend_fontsize,
          loc='upper center', # Anchor point on the legend
          bbox_to_anchor=(0.5, -0.18), # Position: x=center, y=below axes. Adjust -0.18 as needed.
          frameon=True, facecolor='white', framealpha=0.85, ncol=num_legend_cols)

ax.grid(True, which="major", ls="-", alpha=0.4, color='lightgray', zorder=0, linewidth=0.8)

# << ADJUST SUBPLOT PARAMETERS FOR WIDE PLOT & LEGEND BELOW >>
plt.subplots_adjust(left=0.06, right=0.96, top=0.92, bottom=0.25) # Increased bottom margin for legend

pdf_filename = "/home/gelmkaiel/Floaty/Nature Machine Intelligence/Figures/power_vs_mass_kg_wide.pdf"
plt.savefig(pdf_filename, format='pdf', dpi=300)
print(f"Figure saved as {pdf_filename}")
plt.show()