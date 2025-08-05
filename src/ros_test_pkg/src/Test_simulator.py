import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.animation import FuncAnimation

class Speaker:
    def __init__(self, x, y, frequency=440, amplitude=1.0, phase=0.0):
        self.x = x
        self.y = y
        self.frequency = frequency
        self.amplitude = amplitude
        self.phase = phase

    def wave(self, X, Y, t, v=343):
        """Return the wave from this speaker at time t over grid (X, Y)."""
        r = np.sqrt((X - self.x)**2 + (Y - self.y)**2)
        k = 2 * np.pi * self.frequency / v
        return self.amplitude * np.sin(2 * np.pi * self.frequency * t - k * r + self.phase)


class SoundWave2DSimulator:
    def __init__(self, grid_size=100, extent=10, time_step=0.01):
        self.grid_size = grid_size
        self.extent = extent
        self.time_step = time_step
        self.t = 0
        self.speakers = []

        x = np.linspace(0, extent, grid_size)
        y = np.linspace(0, extent, grid_size)
        self.X, self.Y = np.meshgrid(x, y)

        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.im = self.ax.imshow(np.zeros_like(self.X), extent=[0, extent, 0, extent], cmap='RdBu', vmin=-2, vmax=2, origin='lower')
        self.ax.set_title("Left Click: Add Speaker | Right Click: Add Line Array")
        self.fig.colorbar(self.im, ax=self.ax, label="Pressure")

        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        self._setup_sliders()
        self.anim = FuncAnimation(self.fig, self.update, interval=30, blit=False)
        self._setup_restart_button()


    def _setup_sliders(self):
        # Frequency
        ax_freq = plt.axes([0.25, 0.02, 0.5, 0.02])
        self.slider_freq = Slider(ax_freq, 'Frequency (Hz)', 50, 2000, valinit=440)

        # Amplitude
        ax_amp = plt.axes([0.25, 0.05, 0.5, 0.02])
        self.slider_amp = Slider(ax_amp, 'Amplitude', 0.1, 2.0, valinit=1.0)

        # Phase
        ax_phase = plt.axes([0.25, 0.08, 0.5, 0.02])
        self.slider_phase = Slider(ax_phase, 'Phase (rad)', 0, 2*np.pi, valinit=0)

        # Number of speakers in array
        ax_count = plt.axes([0.25, 0.11, 0.5, 0.02])
        self.slider_count = Slider(ax_count, 'Array Size', 2, 20, valinit=5, valstep=1)

        # Spacing in array
        ax_spacing = plt.axes([0.25, 0.14, 0.5, 0.02])
        self.slider_spacing = Slider(ax_spacing, 'Spacing', 0.05, 1.0, valinit=0.2)

        # Phase step per speaker in array
        ax_phase_step = plt.axes([0.25, 0.17, 0.5, 0.02])
        self.slider_phase_step = Slider(ax_phase_step, 'Phase Step (rad)', 0, 2*np.pi, valinit=0.0)


    def restart_simulation(self, event):
        self.speakers.clear()
        self.t = 0
        self.im.set_data(np.zeros_like(self.X))
        print("Simulation restarted.")
        self.fig.canvas.draw_idle()

    def _setup_restart_button(self):
        ax_reset = plt.axes([0.8, 0.88, 0.15, 0.05])
        self.button_reset = Button(ax_reset, 'Restart')
        self.button_reset.on_clicked(self.restart_simulation)

    def on_click(self, event):
        if event.inaxes != self.ax:
            return

        x, y = event.xdata, event.ydata
        freq = self.slider_freq.val
        amp = self.slider_amp.val
        phase = self.slider_phase.val

        if event.button == 1:
            # Left click: single speaker
            self.speakers.append(Speaker(x, y, freq, amp, phase))
            print(f"Added single speaker at ({x:.2f}, {y:.2f})")
        elif event.button == 3:
            # Right click: line array
            count = int(self.slider_count.val)
            spacing = self.slider_spacing.val
            phase_step = self.slider_phase_step.val
            base_phase = self.slider_phase.val

            start_x = x - (count - 1) * spacing / 2

            for i in range(count):
                sx = start_x + i * spacing
                speaker_phase = base_phase + i * phase_step
                self.speakers.append(Speaker(sx, y, freq, amp, speaker_phase))
            print(f"Added line array of {count} speakers centered at ({x:.2f}, {y:.2f}) with phase step {phase_step:.2f}")


    def update(self, frame):
        self.t += self.time_step
        field = np.zeros_like(self.X)
        for speaker in self.speakers:
            field += speaker.wave(self.X, self.Y, self.t)
        self.im.set_data(field)
        return [self.im]

    def run(self):
        plt.show()


# Run the simulator
if __name__ == "__main__":
    sim = SoundWave2DSimulator()
    sim.run()
