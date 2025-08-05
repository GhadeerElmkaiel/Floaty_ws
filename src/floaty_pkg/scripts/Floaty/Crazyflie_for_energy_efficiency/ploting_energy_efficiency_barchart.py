import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib.container import BarContainer # Import BarContainer directly

# --- (Keep the rest of your data and setup code the same) ---

# --- Data for the plot (Mass in Grams, Power in Watts) ---
# ... (your data dictionary here) ...
data = {
    # Drones
    'Floaty': {'mass_g': 340, 'power_updraft_W': 3.4, 'color': 'green', 'sp_updraft_W_kg': (3.4 / 0.340)}, # Specific power for updraft
    'Crazyflie 2.1': {'mass_g': 27, 'power_still_W': 8.0, 'power_updraft_W': 6.6, 'color': 'dodgerblue',
                      'sp_still_W_kg': (8.0 / 0.027), 'sp_updraft_W_kg': (6.6 / 0.027)},
    'Custom Quad': {'mass_g': 950, 'power_still_W': 137.75, 'power_updraft_W': 64.6, 'color': 'firebrick',
                       'sp_still_W_kg': (137.75 / 0.950), 'sp_updraft_W_kg': (64.6 / 0.950)}, # Renamed from Custom Quadcopter for brevity

    'DJI Mini 4 Pro': {'mass_g': 249, 'power_still_W': 33.4, 'color': 'purple', 'sp_still_W_kg': (33.4 / 0.249)},
    'DJI Air 3': {'mass_g': 720, 'power_still_W': 81.6, 'color': 'orange', 'sp_still_W_kg': (81.6 / 0.720)},
    'DJI Mavic 3 Pro': {'mass_g': 958, 'power_still_W': 107.4, 'color': 'teal', 'sp_still_W_kg': (107.4 / 0.958)},
    'DJI Avata': {'mass_g': 410, 'power_still_W': 119, 'color': 'magenta', 'sp_still_W_kg': (119 / 0.410)},

    # Helicopters & eVTOLs
    'Robinson R22': {'mass_g': 622 * 1000, 'power_still_W': 77.5 * 1000, 'color': '#FFC300', 'sp_still_W_kg': (77.5 * 1000 / (622))}, # Already in kW/kg essentially
    'Bell 206B': {'mass_g': 1451 * 1000, 'power_still_W': 165 * 1000, 'color': '#FF5733', 'sp_still_W_kg': (165 * 1000 / (1451))}, # Shortened name
    'Airbus H125': {'mass_g': 2250 * 1000, 'power_still_W': 415 * 1000, 'color': '#C70039', 'sp_still_W_kg': (415 * 1000 / (2250))},
    'UH-60M Black Hawk': {'mass_g': 9979 * 1000, 'power_still_W': 1700 * 1000, 'color': '#900C3F', 'sp_still_W_kg': (1700 * 1000 / (9979))},
    'Joby S4 (eVTOL)': {'mass_g': 2177 * 1000, 'power_still_W': 350 * 1000, 'color': '#2ECC71', 'sp_still_W_kg': (350 * 1000 / (2177))},
}


# --- Prepare data for bar chart ---
vehicle_names = list(data.keys())
sp_still_values = []
# sp_updraft_values = [] # Not strictly needed if plotting updraft separately for grouped
colors = []
# hatches = [] # Not strictly needed if applying hatches directly

for name in vehicle_names:
    props = data[name]
    colors.append(props['color'])
    sp_still_values.append(props.get('sp_still_W_kg', np.nan))


# --- Plotting the Bar Chart ---
sns.set_theme(style="whitegrid")
fig, ax = plt.subplots(figsize=(14, 8))

bar_width = 0.35
index = np.arange(len(vehicle_names))

# Plot "Still Air / Hover" bars
# We store the BarContainer object returned by ax.bar
bars_still_container = ax.bar(index - bar_width/2, sp_still_values, bar_width,
                              label='Still Air / Hover', color=[c if not np.isnan(s) else 'none' for c, s in zip(colors, sp_still_values)],
                              edgecolor='black')

# Plot "Updraft" bars
updraft_plot_indices = []
updraft_plot_values = []
updraft_plot_colors = []
updraft_plot_hatches = []
floaty_bar = None # To store Floaty's bar object if plotted

for i, name in enumerate(vehicle_names):
    if name == 'Floaty':
        floaty_sp_val = data[name]['sp_updraft_W_kg']
        if not np.isnan(floaty_sp_val):
            # Plot Floaty's updraft as a single bar in its position
            # Store the BarContainer object (which will contain one bar)
            floaty_bar_container = ax.bar(index[i], floaty_sp_val, bar_width * 1.5, # Make it slightly wider
                                 color=data[name]['color'], edgecolor='black', hatch='xx')
            if floaty_bar_container: # Check if bar was actually plotted
                floaty_bar = floaty_bar_container[0] # Get the actual bar artist
    elif name in ['Crazyflie 2.1', 'Custom Quad']:
        updraft_val = data[name].get('sp_updraft_W_kg')
        if updraft_val is not None and not np.isnan(updraft_val):
            updraft_plot_indices.append(index[i] + bar_width/2)
            updraft_plot_values.append(updraft_val)
            updraft_plot_colors.append(data[name]['color'])
            updraft_plot_hatches.append('//')

bars_updraft_container = None # Initialize
if updraft_plot_indices: # Only plot if there's data for it
    bars_updraft_container = ax.bar(updraft_plot_indices, updraft_plot_values, bar_width,
                                    label='Updraft', color=updraft_plot_colors, edgecolor='black', hatch=updraft_plot_hatches)


# --- Chart Customization ---
ax.set_xlabel('Vehicle', fontsize=14, fontweight='bold')
ax.set_ylabel('Specific Power Consumption (W/kg)', fontsize=14, fontweight='bold')
ax.set_title('Specific Power Consumption (Hover/Updraft)', fontsize=16, fontweight='bold')
ax.set_xticks(index)
ax.set_xticklabels(vehicle_names, rotation=45, ha="right", fontsize=10)
ax.tick_params(axis='y', labelsize=12)

# Add value labels on top of bars
def autolabel_bars(bar_container_obj):
    """Attach a text label above each bar in *bar_container_obj*, displaying its height."""
    if bar_container_obj is None: # Check if the container is None (e.g., no updraft bars plotted)
        return
    if not isinstance(bar_container_obj, BarContainer): # Check the type correctly
        print(f"Warning: autolabel_bars received an object of type {type(bar_container_obj)}, expected BarContainer.")
        return

    for bar in bar_container_obj: # Iterate through bars in the BarContainer
        height = bar.get_height()
        if not np.isnan(height) and height > 0:
            ax.annotate(f'{height:.0f}',
                        xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom', fontsize=8)

autolabel_bars(bars_still_container)
autolabel_bars(bars_updraft_container)

# Label Floaty's bar if it was plotted and stored
if floaty_bar:
    height = floaty_bar.get_height()
    if not np.isnan(height) and height > 0:
        ax.annotate(f'{height:.0f}',
                    xy=(floaty_bar.get_x() + floaty_bar.get_width() / 2, height),
                    xytext=(0, 3), textcoords="offset points",
                    ha='center', va='bottom', fontsize=8)


# Create a more robust legend
legend_handles = []
legend_labels = []

if any(not np.isnan(v) for v in sp_still_values):
    legend_handles.append(plt.Rectangle((0, 0), 1, 1, color='gray', edgecolor='black', label='Still Air / Hover'))
    legend_labels.append('Still Air / Hover')

if updraft_plot_values: # If any grouped updraft bars were plotted
    legend_handles.append(plt.Rectangle((0, 0), 1, 1, color='gray', edgecolor='black', hatch='//', label='Updraft (Grouped)'))
    legend_labels.append('Updraft (Crazyflie/Custom Quad)')

if not np.isnan(data['Floaty'].get('sp_updraft_W_kg')):
    legend_handles.append(plt.Rectangle((0, 0), 1, 1, color=data['Floaty']['color'], edgecolor='black', hatch='xx', label='Floaty (Updraft Only)'))
    legend_labels.append('Floaty (Updraft Only)')


ax.legend(handles=legend_handles, labels=legend_labels, fontsize=10, loc='upper right')
ax.grid(True, axis='y', linestyle='--', alpha=0.7)
plt.tight_layout()

# --- Save the figure ---
pdf_filename = "/home/gelmkaiel/Floaty/Nature Machine Intelligence/Figures/specific_power_barchart.pdf"
plt.savefig(pdf_filename, format='pdf', dpi=300)
print(f"Figure saved as {pdf_filename}")

plt.show()