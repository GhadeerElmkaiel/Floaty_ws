# import matplotlib.pyplot as plt
# import numpy as np
# import seaborn as sns
# from matplotlib.ticker import LogLocator, FuncFormatter

# # --- Data for the plot (Mass in Grams) ---
# # Vehicle: [Mass (g), Power (W) in Still Air, Power (W) in Updraft (~10m/s)]
# data = {
#     'Floaty': {'mass_g': 340, 'power_still_W': None, 'power_updraft_W': 3.4, 'color': 'green', 'label_offset': (10, 0)},
#     'Crazyflie 2.1': {'mass_g': 27, 'power_still_W': 8.0, 'power_updraft_W': 6.6, 'color': 'dodgerblue', 'label_offset': (10, -5)}, # Using a specific blue
#     'Custom Quadcopter': {'mass_g': 950, 'power_still_W': 137.75, 'power_updraft_W': 64.6, 'color': 'firebrick', 'label_offset': (10, 0)}, # Using a specific red
# }

# # --- Plotting ---
# sns.set_theme(style="whitegrid", rc={"grid.linestyle": ":", "grid.alpha": 0.7})
# plt.figure(figsize=(10, 7))

# # Define markers
# marker_still_air = '+'
# marker_updraft = '*'
# marker_floaty = 'o' # Floaty is always in updraft

# # Plotting the data points and labels
# for name, props in data.items():
#     label_x_offset = props.get('label_offset', (5,5))[0]
#     label_y_offset = props.get('label_offset', (5,5))[1]
#     current_marker_still = marker_still_air
#     current_marker_updraft = marker_updraft

#     if name == 'Floaty':
#         current_marker_updraft = marker_floaty # Floaty specific marker

#     # Plot still air data if available
#     if props['power_still_W'] is not None:
#         plt.scatter(props['mass_g'], props['power_still_W'],
#                     s=180, # Slightly larger +
#                     label=f'{name} (Still Air)',
#                     color=props['color'], marker=current_marker_still, alpha=0.9, ec=props['color'], zorder=5, linewidths=1.5)
#         # plt.annotate(name, (props['mass_g'], props['power_still_W']),
#         #              textcoords="offset points", xytext=(label_x_offset, label_y_offset - (10 if props['power_updraft_W'] else 0) ), # Adjust offset if both points exist
#         #              ha='left', fontsize=9.5, color=props['color'], alpha=0.9, zorder=6)

#     # Plot updraft data if available
#     if props['power_updraft_W'] is not None:
#         plt.scatter(props['mass_g'], props['power_updraft_W'],
#                     s=150, # Standard circle size
#                     label=f'{name} (Updraft)',
#                     color=props['color'], marker=current_marker_updraft, alpha=1.0, ec=props['color'], zorder=5)
#         # # Only label near updraft point if still air point also exists for that vehicle, or if it's Floaty
#         # if props['power_still_W'] is not None or name == 'Floaty':
#         #      plt.annotate(name, (props['mass_g'], props['power_updraft_W']),
#         #                  textcoords="offset points", xytext=(label_x_offset, label_y_offset + (10 if props['power_still_W'] else 0)), # Adjust offset
#         #                  ha='left', fontsize=9.5, color=props['color'], zorder=6)
#         # elif props['power_still_W'] is None and name != 'Floaty': # Should not happen with current data, but good for robustness
#         #      plt.annotate(name, (props['mass_g'], props['power_updraft_W']),
#         #                  textcoords="offset points", xytext=(label_x_offset, label_y_offset),
#         #                  ha='left', fontsize=9.5, color=props['color'], zorder=6)


# # --- Lines of Constant Specific Power ---
# mass_range_g = np.logspace(np.log10(10), np.log10(10000), 100)

# specific_power_floaty_W_g = 0.01 # 10 W/kg
# power_floaty_line = specific_power_floaty_W_g * mass_range_g
# plt.plot(mass_range_g, power_floaty_line, 'k--', alpha=0.9, linewidth=2, zorder=3,
#          label=f'{specific_power_floaty_W_g*1000:.0f} W/kg (Floaty)') # Simplified label

# specific_power_ref_W_g = 0.1 # 100 W/kg
# power_ref_line = specific_power_ref_W_g * mass_range_g
# plt.plot(mass_range_g, power_ref_line, 'dimgrey', linestyle=':', alpha=0.8, linewidth=1.5, zorder=3,
#          label=f'{specific_power_ref_W_g*1000:.0f} W/kg (Typical Multirotor)')

# # --- Plot Customization ---
# plt.xscale('log')
# plt.yscale('log')

# plt.xlabel('Mass (g)', fontsize=14, fontweight='bold')
# plt.ylabel('Power Consumption (W)', fontsize=14, fontweight='bold')
# plt.title('Power Consumption vs. Mass for Aerial Robots', fontsize=16, fontweight='bold')

# def plain_num_formatter(x, pos):
#     if x >= 1: return f'{x:.0f}'
#     else: return f'{x:.2f}' # Show decimals for values < 1 if needed

# ax = plt.gca()
# ax.xaxis.set_major_locator(LogLocator(base=10.0, numticks=15))
# ax.xaxis.set_major_formatter(FuncFormatter(plain_num_formatter))
# ax.xaxis.set_minor_formatter(FuncFormatter(lambda x,pos: ""))

# ax.yaxis.set_major_locator(LogLocator(base=10.0, numticks=15))
# ax.yaxis.set_major_formatter(FuncFormatter(plain_num_formatter))
# ax.yaxis.set_minor_formatter(FuncFormatter(lambda x,pos: ""))

# valid_powers = [p for d in data.values() for p in [d['power_still_W'], d['power_updraft_W']] if p is not None]
# min_power_val = min(valid_powers) if valid_powers else 1
# max_power_val = max(valid_powers) if valid_powers else 100
# min_mass_val = min(d['mass_g'] for d in data.values())
# max_mass_val = max(d['mass_g'] for d in data.values())

# # plt.xlim(min_mass_val * 0.3, max_mass_val * 3) # Adjusted padding
# # plt.ylim(min_power_val * 0.3, max_power_val * 3) # Adjusted padding
# plt.xlim(10, 2000) # Adjusted padding
# plt.ylim(1, 500) # Adjusted padding

# # Create custom legend handles for clarity
# from matplotlib.lines import Line2D
# legend_elements = [
#     Line2D([0], [0], marker=marker_floaty, color='w', label='Floaty (Updraft)', markerfacecolor='green', markeredgecolor='green', markersize=10),
#     Line2D([0], [0], marker=marker_still_air, color='w', label='Crazyflie (Still Air)', markerfacecolor='dodgerblue', markeredgecolor='dodgerblue', markersize=12, linestyle='None', markeredgewidth=1.5),
#     Line2D([0], [0], marker=marker_updraft, color='w', label='Crazyflie (Updraft)', markerfacecolor='dodgerblue', markeredgecolor='dodgerblue', markersize=12, linestyle='None', markeredgewidth=1.5),
#     # Crazyflie updraft not plotted with current data
#     Line2D([0], [0], marker=marker_still_air, color='w', label='Custom Quad (Still Air)', markerfacecolor='firebrick', markeredgecolor='firebrick', markersize=12, linestyle='None', markeredgewidth=1.5),
#     Line2D([0], [0], marker=marker_updraft, color='w', label='Custom Quad (Updraft)', markerfacecolor='firebrick', markeredgecolor='firebrick', markersize=10),
#     Line2D([0], [0], color='k', lw=2, linestyle='--', label='10 W/kg (Floaty Specific Power)'),
#     Line2D([0], [0], color='dimgrey', lw=1.5, linestyle=':', label='100 W/kg (Typical Multirotor)')
# ]
# ax.legend(handles=legend_elements, fontsize=10, loc='upper left', frameon=True, facecolor='white', framealpha=0.9)


# plt.xticks(fontsize=12)
# plt.yticks(fontsize=12)

# ax.grid(True, which="major", ls="-", alpha=0.5, color='lightgray', zorder=0)
# ax.grid(False, which="minor")

# plt.tight_layout()


# # --- Save the figure as a PDF ---
# pdf_filename = "/home/floaty/Floaty/Nature Machine Intelligence/Figures/power_vs_mass_comparison.pdf"
# plt.savefig(pdf_filename, format='pdf', bbox_inches='tight', dpi=300)
# print(f"Figure saved as {pdf_filename}")

# plt.show()


# ==================================================

# import matplotlib.pyplot as plt
# import numpy as np
# import seaborn as sns
# from matplotlib.ticker import LogLocator, FuncFormatter
# from matplotlib.lines import Line2D
# from matplotlib.patches import Patch # Import Patch for legend

# # --- Data for the plot (Mass in Grams) ---
# # Vehicle: [Mass (g), Power (W) in Still Air, Power (W) in Updraft (~10m/s)]
# data = {
#     'Floaty': {'mass_g': 340, 'power_still_W': None, 'power_updraft_W': 3.4, 'color': 'green', 'label_offset': (10, 0)},
#     'Crazyflie 2.1': {'mass_g': 27, 'power_still_W': 8.0, 'power_updraft_W': 6.6, 'color': 'dodgerblue', 'label_offset': (10, -5)},
#     'Custom Quadcopter': {'mass_g': 950, 'power_still_W': 137.75, 'power_updraft_W': 64.6, 'color': 'firebrick', 'label_offset': (10, 0)},
#     # --- ADDED DJI DRONES ---
#     'DJI Mini 4 Pro': {'mass_g': 249, 'power_still_W': 33.4, 'power_updraft_W': None, 'color': 'purple', 'label_offset': (10, 0)},
#     'DJI Air 3': {'mass_g': 720, 'power_still_W': 81.6, 'power_updraft_W': None, 'color': 'orange', 'label_offset': (10, 0)},
#     'DJI Mavic 3 Pro': {'mass_g': 958, 'power_still_W': 107.4, 'power_updraft_W': None, 'color': 'teal', 'label_offset': (10, 0)},
#     'DJI Avata': {'mass_g': 410, 'power_still_W': 119, 'power_updraft_W': None, 'color': 'magenta', 'label_offset': (10, 0)},
# }

# # --- Plotting ---
# sns.set_theme(style="whitegrid", rc={"grid.linestyle": ":", "grid.alpha": 0.7})
# plt.figure(figsize=(12, 8))

# # Define markers
# marker_still_air = '+'
# marker_updraft = '*'
# marker_floaty = 'o'

# # --- Lines of Constant Specific Power ---
# mass_range_g = np.logspace(np.log10(10), np.log10(3000), 100)

# specific_power_floaty_W_g = 0.01 # 10 W/kg
# power_floaty_line = specific_power_floaty_W_g * mass_range_g

# specific_power_ref_W_g = 0.1 # 100 W/kg
# power_ref_line = specific_power_ref_W_g * mass_range_g

# specific_power_high_W_g = 0.3 # 300 W/kg
# power_high_line = specific_power_high_W_g * mass_range_g

# # --- Highlight Area Between 100 W/kg and 300 W/kg ---
# fill_color = 'lightgray'
# fill_alpha = 0.4
# plt.fill_between(mass_range_g, power_ref_line, power_high_line,
#                  color=fill_color, alpha=fill_alpha, zorder=1, # Ensure it's behind lines and points
#                  label='100-300 W/kg Range (Typical to High Perf. Multirotors)')


# # Plotting the specific power lines
# plt.plot(mass_range_g, power_floaty_line, 'k--', alpha=0.9, linewidth=2, zorder=3,
#          label=f'{specific_power_floaty_W_g*1000:.0f} W/kg (Floaty Target)')
# plt.plot(mass_range_g, power_ref_line, 'dimgrey', linestyle=':', alpha=0.8, linewidth=1.5, zorder=3,
#          label=f'{specific_power_ref_W_g*1000:.0f} W/kg (Typical Multirotor)')
# plt.plot(mass_range_g, power_high_line, 'slategray', linestyle=':', alpha=0.8, linewidth=1.5, zorder=3, # Changed color for better visibility
#          label=f'{specific_power_high_W_g*1000:.0f} W/kg (High Perf. Multirotor)')


# # Plotting the data points and labels
# for name, props in data.items():
#     label_x_offset = props.get('label_offset', (5,5))[0]
#     label_y_offset = props.get('label_offset', (5,5))[1]
#     current_marker_still = marker_still_air
#     current_marker_updraft = marker_updraft

#     if name == 'Floaty':
#         current_marker_updraft = marker_floaty

#     # Plot still air data if available
#     if props['power_still_W'] is not None:
#         plt.scatter(props['mass_g'], props['power_still_W'],
#                     s=180,
#                     label=f'{name} (Still Air/Hover)',
#                     color=props['color'], marker=current_marker_still, alpha=0.9, ec=props['color'], zorder=5, linewidths=1.5)

#     # Plot updraft data if available
#     if props['power_updraft_W'] is not None:
#         plt.scatter(props['mass_g'], props['power_updraft_W'],
#                     s=150,
#                     label=f'{name} (Updraft)',
#                     color=props['color'], marker=current_marker_updraft, alpha=1.0, ec=props['color'], zorder=5)


# # --- Plot Customization ---
# plt.xscale('log')
# plt.yscale('log')

# plt.xlabel('Mass (g)', fontsize=14, fontweight='bold')
# plt.ylabel('Power Consumption (W)', fontsize=14, fontweight='bold')
# plt.title('Power Consumption vs. Mass for Aerial Robots', fontsize=16, fontweight='bold')

# def plain_num_formatter(x, pos):
#     if x >= 1: return f'{x:.0f}'
#     if x >= 0.1: return f'{x:.1f}'
#     else: return f'{x:.2f}'

# ax = plt.gca()
# ax.xaxis.set_major_locator(LogLocator(base=10.0, numticks=15))
# ax.xaxis.set_major_formatter(FuncFormatter(plain_num_formatter))
# ax.xaxis.set_minor_formatter(FuncFormatter(lambda x,pos: ""))

# ax.yaxis.set_major_locator(LogLocator(base=10.0, numticks=15))
# ax.yaxis.set_major_formatter(FuncFormatter(plain_num_formatter))
# ax.yaxis.set_minor_formatter(FuncFormatter(lambda x,pos: ""))

# all_masses = [props['mass_g'] for props in data.values()]
# all_powers_still = [props['power_still_W'] for props in data.values() if props['power_still_W'] is not None]
# all_powers_updraft = [props['power_updraft_W'] for props in data.values() if props['power_updraft_W'] is not None]
# all_valid_powers = all_powers_still + all_powers_updraft

# min_power_val = min(all_valid_powers) if all_valid_powers else 1
# max_power_val = max(all_valid_powers) if all_valid_powers else 100
# min_mass_val = min(all_masses) if all_masses else 10
# max_mass_val = max(all_masses) if all_masses else 1000

# plt.xlim(min_mass_val * 0.5, max_mass_val * 1.5)
# plt.ylim(min_power_val * 0.5, max_power_val * 1.5)


# # Create custom legend handles for clarity
# legend_elements = [
#     # Existing vehicles
#     Line2D([0], [0], marker=marker_floaty, color='w', label='Floaty (Updraft)', markerfacecolor='green', markeredgecolor='green', markersize=10),
#     Line2D([0], [0], marker=marker_still_air, color='w', label='Crazyflie (Still Air)', markerfacecolor='dodgerblue', markeredgecolor='dodgerblue', markersize=12, linestyle='None', markeredgewidth=1.5),
#     Line2D([0], [0], marker=marker_updraft, color='w', label='Crazyflie (Updraft)', markerfacecolor='dodgerblue', markeredgecolor='dodgerblue', markersize=10),
#     Line2D([0], [0], marker=marker_still_air, color='w', label='Custom Quad (Still Air)', markerfacecolor='firebrick', markeredgecolor='firebrick', markersize=12, linestyle='None', markeredgewidth=1.5),
#     Line2D([0], [0], marker=marker_updraft, color='w', label='Custom Quad (Updraft)', markerfacecolor='firebrick', markeredgecolor='firebrick', markersize=10),

#     # DJI Drones (Still Air / Hover)
#     Line2D([0], [0], marker=marker_still_air, color='w', label='DJI Mini 4 Pro (Hover)', markerfacecolor=data['DJI Mini 4 Pro']['color'], markeredgecolor=data['DJI Mini 4 Pro']['color'], markersize=12, linestyle='None', markeredgewidth=1.5),
#     Line2D([0], [0], marker=marker_still_air, color='w', label='DJI Air 3 (Hover)', markerfacecolor=data['DJI Air 3']['color'], markeredgecolor=data['DJI Air 3']['color'], markersize=12, linestyle='None', markeredgewidth=1.5),
#     Line2D([0], [0], marker=marker_still_air, color='w', label='DJI Mavic 3 Pro (Hover)', markerfacecolor=data['DJI Mavic 3 Pro']['color'], markeredgecolor=data['DJI Mavic 3 Pro']['color'], markersize=12, linestyle='None', markeredgewidth=1.5),
#     Line2D([0], [0], marker=marker_still_air, color='w', label='DJI Avata (Hover)', markerfacecolor=data['DJI Avata']['color'], markeredgecolor=data['DJI Avata']['color'], markersize=12, linestyle='None', markeredgewidth=1.5),

#     # Specific Power Lines
#     Line2D([0], [0], color='k', lw=2, linestyle='--', label='10 W/kg (Floaty Target)'),
#     Line2D([0], [0], color='dimgrey', lw=1.5, linestyle=':', label='100 W/kg (Typical Multirotor)'),
#     Line2D([0], [0], color='slategray', lw=1.5, linestyle=':', label='300 W/kg (High Perf. Multirotor)'), # Matched color

#     # Shaded Area
#     Patch(facecolor=fill_color, edgecolor=fill_color, alpha=fill_alpha, label='100-300 W/kg Range')
# ]
# ax.legend(handles=legend_elements, fontsize=9, loc='upper left', frameon=True, facecolor='white', framealpha=0.9, ncol=1)


# plt.xticks(fontsize=12)
# plt.yticks(fontsize=12)

# ax.grid(True, which="major", ls="-", alpha=0.5, color='lightgray', zorder=0)
# ax.grid(True, which="minor", ls=":", alpha=0.3, color='lightgray', zorder=0)

# plt.tight_layout(rect=[0, 0, 0.85, 1]) # Adjust if legend is outside, or remove rect if inside

# # --- Save the figure as a PDF ---
# pdf_filename = "/home/floaty/Floaty/Nature Machine Intelligence/Figures/power_vs_mass_comparison_with_dji_shaded.pdf" # Updated filename
# plt.savefig(pdf_filename, format='pdf', bbox_inches='tight', dpi=300)
# print(f"Figure saved as {pdf_filename}")

# plt.show()


# ==============================================


import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib.ticker import LogLocator, FuncFormatter
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

# --- Data for the plot (Mass in Grams) ---
data = {
    'Floaty': {'mass_g': 340, 'power_still_W': None, 'power_updraft_W': 3.4, 'color': 'green', 'label_offset': (10, 0)},
    'Crazyflie 2.1': {'mass_g': 27, 'power_still_W': 8.0, 'power_updraft_W': 6.6, 'color': 'dodgerblue', 'label_offset': (10, -5)},
    'Custom Quadcopter': {'mass_g': 950, 'power_still_W': 137.75, 'power_updraft_W': 64.6, 'color': 'firebrick', 'label_offset': (10, 0)},
    # --- DJI DRONES with custom markers ---
    'DJI Mini 4 Pro': {'mass_g': 249, 'power_still_W': 33.4, 'power_updraft_W': None, 'color': 'purple', 'label_offset': (10, 0), 'marker_override': '^'}, # Square
    'DJI Air 3': {'mass_g': 720, 'power_still_W': 81.6, 'power_updraft_W': None, 'color': 'orange', 'label_offset': (10, 0), 'marker_override': '^'},  # Diamond
    'DJI Mavic 3 Pro': {'mass_g': 958, 'power_still_W': 107.4, 'power_updraft_W': None, 'color': 'teal', 'label_offset': (10, 0), 'marker_override': '^'},   # Triangle Down
    'DJI Avata': {'mass_g': 410, 'power_still_W': 119, 'power_updraft_W': None, 'color': 'magenta', 'label_offset': (10, 0), 'marker_override': '^'}, # Triangle Up
}

# --- Plotting ---
sns.set_theme(style="whitegrid", rc={"grid.linestyle": ":", "grid.alpha": 0.7})
plt.figure(figsize=(7, 5.5)) # Adjusted figsize slightly for potentially better legend fit

title_fontsize = 12
axis_label_fontsize = 10
tick_label_fontsize = 8
legend_fontsize = 7.5

scatter_marker_size_default = 90  # General size for new DJI markers and updraft markers
scatter_marker_size_plus = 100    # Size for '+' marker
legend_marker_size = 6
line_width_main = 1.5
line_width_secondary = 1.2

# Default markers (can be overridden)
marker_still_air_default = '+' # Default for non-DJI still air
marker_updraft_default = '*'
marker_floaty = 'o'

# --- Lines of Constant Specific Power ---
mass_range_g = np.logspace(np.log10(10), np.log10(3000), 100)
specific_power_floaty_W_g = 0.01
power_floaty_line = specific_power_floaty_W_g * mass_range_g
specific_power_ref_W_g = 0.1
power_ref_line = specific_power_ref_W_g * mass_range_g
specific_power_high_W_g = 0.3
power_high_line = specific_power_high_W_g * mass_range_g

# --- Highlight Area ---
fill_color = 'lightgray'
fill_alpha = 0.3
plt.fill_between(mass_range_g, power_ref_line, power_high_line,
                 color=fill_color, alpha=fill_alpha, zorder=1,
                 label='100-300 W/kg Range')

# Plotting specific power lines
plt.plot(mass_range_g, power_floaty_line, 'k--', alpha=0.9, linewidth=line_width_main, zorder=3, label='10 W/kg')
plt.plot(mass_range_g, power_ref_line, 'dimgrey', linestyle=':', alpha=0.8, linewidth=line_width_secondary, zorder=3, label='100 W/kg')
plt.plot(mass_range_g, power_high_line, 'slategray', linestyle=':', alpha=0.8, linewidth=line_width_secondary, zorder=3, label='300 W/kg')

# Plotting data points
for name, props in data.items():
    # Determine marker for still air/hover
    current_marker_still = props.get('marker_override', marker_still_air_default)
    # Determine marker for updraft
    current_marker_updraft = marker_updraft_default
    if name == 'Floaty':
        current_marker_updraft = marker_floaty

    # Determine marker size
    # '+' marker often looks smaller, so we might give it a slightly larger 's' value
    marker_size_still = scatter_marker_size_plus if current_marker_still == '+' else scatter_marker_size_default
    marker_size_updraft = scatter_marker_size_default


    if props['power_still_W'] is not None:
        plt.scatter(props['mass_g'], props['power_still_W'],
                    s=marker_size_still,
                    label=f'{name} (Still/Hover)',
                    color=props['color'], marker=current_marker_still, alpha=0.9, ec=props['color'], zorder=5,
                    linewidths=1.2 if current_marker_still == '+' else 0.5) # Thinner edge for filled markers

    if props['power_updraft_W'] is not None:
        plt.scatter(props['mass_g'], props['power_updraft_W'],
                    s=marker_size_updraft,
                    label=f'{name} (Updraft)',
                    color=props['color'], marker=current_marker_updraft, alpha=1.0, ec=props['color'], zorder=5,
                    linewidths=1.0 if current_marker_updraft == '*' else 0.5)

# --- Plot Customization ---
plt.xscale('log')
plt.yscale('log')
plt.xlabel('Mass (g)', fontsize=axis_label_fontsize, fontweight='bold')
plt.ylabel('Power Consumption (W)', fontsize=axis_label_fontsize, fontweight='bold')
plt.title('Power Consumption vs. Mass', fontsize=title_fontsize, fontweight='bold')

def plain_num_formatter(x, pos):
    if x >= 10: return f'{x:.0f}'
    if x >= 1: return f'{x:.0f}'
    if x >= 0.1: return f'{x:.1f}'
    else: return f'{x:.2f}'

ax = plt.gca()
ax.xaxis.set_major_locator(LogLocator(base=10.0, numticks=10))
ax.xaxis.set_major_formatter(FuncFormatter(plain_num_formatter))
ax.xaxis.set_minor_formatter(FuncFormatter(lambda x,pos: ""))
ax.yaxis.set_major_locator(LogLocator(base=10.0, numticks=10))
ax.yaxis.set_major_formatter(FuncFormatter(plain_num_formatter))
ax.yaxis.set_minor_formatter(FuncFormatter(lambda x,pos: ""))
plt.xticks(fontsize=tick_label_fontsize)
plt.yticks(fontsize=tick_label_fontsize)

all_masses = [props['mass_g'] for props in data.values()]
all_powers_still = [props['power_still_W'] for props in data.values() if props['power_still_W'] is not None]
all_powers_updraft = [props['power_updraft_W'] for props in data.values() if props['power_updraft_W'] is not None]
all_valid_powers = all_powers_still + all_powers_updraft
if all_masses and all_valid_powers:
    min_power_val = min(all_valid_powers); max_power_val = max(all_valid_powers)
    min_mass_val = min(all_masses); max_mass_val = max(all_masses)
    plt.xlim(min_mass_val * 0.7, max_mass_val * 1.3)
    plt.ylim(min_power_val * 0.7, max_power_val * 1.3)
else:
    plt.xlim(10, 3000); plt.ylim(1, 500)

# Create custom legend handles
legend_elements = [
    # Vehicles
    Line2D([0], [0], marker=marker_floaty, color='w', label='Floaty (Updraft)', markerfacecolor='green', markeredgecolor='green', markersize=legend_marker_size),
    Line2D([0], [0], marker=marker_still_air_default, color='w', label='Crazyflie (Still)', markerfacecolor='dodgerblue', markeredgecolor='dodgerblue', markersize=legend_marker_size+1, linestyle='None', markeredgewidth=1.2),
    Line2D([0], [0], marker=marker_updraft_default, color='w', label='Crazyflie (Updraft)', markerfacecolor='dodgerblue', markeredgecolor='dodgerblue', markersize=legend_marker_size, linestyle='None'),
    Line2D([0], [0], marker=marker_still_air_default, color='w', label='Custom Quad (Still)', markerfacecolor='firebrick', markeredgecolor='firebrick', markersize=legend_marker_size+1, linestyle='None', markeredgewidth=1.2),
    Line2D([0], [0], marker=marker_updraft_default, color='w', label='Custom Quad (Updraft)', markerfacecolor='firebrick', markeredgecolor='firebrick', markersize=legend_marker_size, linestyle='None'),

    # DJI Drones with their new markers
    Line2D([0], [0], marker=data['DJI Mini 4 Pro']['marker_override'], color='w', label='DJI Mini 4 Pro', markerfacecolor=data['DJI Mini 4 Pro']['color'], markeredgecolor=data['DJI Mini 4 Pro']['color'], markersize=legend_marker_size, linestyle='None'),
    Line2D([0], [0], marker=data['DJI Air 3']['marker_override'], color='w', label='DJI Air 3', markerfacecolor=data['DJI Air 3']['color'], markeredgecolor=data['DJI Air 3']['color'], markersize=legend_marker_size, linestyle='None'),
    Line2D([0], [0], marker=data['DJI Mavic 3 Pro']['marker_override'], color='w', label='DJI Mavic 3 Pro', markerfacecolor=data['DJI Mavic 3 Pro']['color'], markeredgecolor=data['DJI Mavic 3 Pro']['color'], markersize=legend_marker_size, linestyle='None'),
    Line2D([0], [0], marker=data['DJI Avata']['marker_override'], color='w', label='DJI Avata', markerfacecolor=data['DJI Avata']['color'], markeredgecolor=data['DJI Avata']['color'], markersize=legend_marker_size, linestyle='None'),

    # Specific Power Lines
    Line2D([0], [0], color='k', lw=line_width_main, linestyle='--', label='10 W/kg'),
    Line2D([0], [0], color='dimgrey', lw=line_width_secondary, linestyle=':', label='100 W/kg'),
    Line2D([0], [0], color='slategray', lw=line_width_secondary, linestyle=':', label='300 W/kg'),
    Patch(facecolor=fill_color, edgecolor='darkgrey', alpha=fill_alpha, label='100-300 W/kg Range')
]
# ax.legend(handles=legend_elements, fontsize=legend_fontsize, loc='upper left',
#           frameon=True, facecolor='white', framealpha=0.85, ncol=2)

# << --- ADJUST LEGEND FOR VERTICAL STACKING (OUTSIDE PLOT) --- >>
ax.legend(handles=legend_elements, fontsize=legend_fontsize,
          loc='center left', # Location of the anchor point of the legend
          bbox_to_anchor=(1.02, 0.5), # Position the anchor point outside plot (x > 1)
          frameon=True, facecolor='white', framealpha=0.85,
          ncol=1) # Single column for vertical stacking

ax.grid(True, which="major", ls="-", alpha=0.4, color='lightgray', zorder=0, linewidth=0.8)
ax.grid(True, which="minor", ls=":", alpha=0.2, color='lightgray', zorder=0, linewidth=0.5)

plt.tight_layout()
pdf_filename = "/home/floaty/Floaty/Nature Machine Intelligence/Figures/power_vs_mass_small_figure_dji_markers.pdf" # Updated filename
plt.savefig(pdf_filename, format='pdf', bbox_inches='tight', dpi=300)
print(f"Figure saved as {pdf_filename}")
plt.show()