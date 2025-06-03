import time
from collections import deque
import os

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.image as mpimg
from matplotlib import gridspec

from ah_wrapper.hand import Hand


class CombinedRealTimePlot:
    def __init__(self, hand):
        self.hand = hand

        # Create a single figure with GridSpec for flexible layout
        self.fig = plt.figure(figsize=(10.5, 10))
        self.fig.canvas.manager.set_window_title("PSYONIC")

        # Setup GridSpec layout: 7 rows x 2 columns (touch: left, motor: right)
        gs = gridspec.GridSpec(
            5, 2, figure=self.fig, width_ratios=[1, 1.2], wspace=0.4
        )

        # Axes for touch plots (7 rows x 1 col on left)
        touch_axes = [self.fig.add_subplot(gs[i, 0]) for i in range(5)]

        # Axes for motor plots (3 rows x 1 col on right starting from row 2)
        motor_axes = [self.fig.add_subplot(gs[i * 2, 1]) for i in range(3)]

        # Load and embed the image for the legend (optional)
        try:
            image_path = os.path.join(
                os.path.dirname(__file__), "touch_sensor_legend_sml.png"
            )
            img = mpimg.imread(image_path)
            self.fig.figimage(img, xo=30, yo=825, zorder=10)
        except FileNotFoundError:
            print("Touch sensor legend image not found.")

        # Initialize both plotters
        self.plot_touch = RealTimePlotTouch(
            hand, fig=self.fig, axes=touch_axes
        )
        self.plot_motors = RealTimePlotMotors(
            hand, fig=self.fig, axes=motor_axes
        )

    def update(self, frame):
        touch_artists = self.plot_touch.update_plot(frame)
        motor_artists = self.plot_motors.update_plot(frame)
        return touch_artists + motor_artists

    def start(self):
        ani = FuncAnimation(
            self.fig,
            self.update,
            interval=10,
            blit=True,
            cache_frame_data=False,
        )
        plt.show()


class RealTimePlotMotors:
    def __init__(
        self, hand, fig=None, axes=None, window_size=5, sample_rate=500
    ):
        self.hand = hand
        self.window_size = window_size
        self.sample_rate = sample_rate
        self.max_samples = (
            window_size * sample_rate
        )  # 5s * 500Hz = 2500 samples
        self.start_time = time.time()

        # Using deque for fast FIFO operations
        self.x_data = deque(maxlen=self.max_samples)
        self.y_data = [
            deque(maxlen=self.max_samples) for _ in range(18)
        ]  # 3 plots with 5 values each

        # Set up three subplots for 5 values each
        if fig is None:
            self.fig, self.axes = plt.subplots(
                3, 1, figsize=(13, 10), sharex=True, num="PSYONIC"
            )
        else:
            self.fig = fig
            self.axes = axes
        self.lines = []
        self.fig.suptitle("Ability Hand", fontsize=16)

        titles = ["Position", "Velocity", "Current"]
        y_values = [
            "Position (" + "\N{DEGREE SIGN}" + ")",
            "Velocity (" + "\N{DEGREE SIGN}" + "per s)",
            "Current (a)",
        ]
        y_ranges = ((0, 110), (-500, 500), (-1, 1))
        for i, ax in enumerate(self.axes):
            lines = [
                ax.plot([], [], label=f"Value {j}")[0]
                for j in range(i * 6, (i + 1) * 6)
            ]
            self.lines.append(lines)
            ax.set_ylim(y_ranges[i][0], y_ranges[i][1])
            ax.set_ylabel(y_values[i])
            ax.set_title(titles[i], fontweight="bold")

        self.axes[-1].set_xlabel("Time (s)")
        plt.subplots_adjust(hspace=0.6)  # Increase hspace for more space
        self.fig.legend(
            self.lines[0],
            ["Index", "Middle", "Ring", "Pinky", "Thb. Flex", "Thb. Rot."],
            loc="upper right",
            fontsize=8,
        )

    def update_plot(self, frame):
        """Updates the plots with the latest data."""
        current_time = time.time() - self.start_time
        self.x_data.append(current_time)

        new_values = []
        hand_pos = self.hand.get_position()
        for i in range(6):
            if hand_pos:
                new_values.append(hand_pos[i])
            else:
                new_values.append(0)
        new_values[-1] *= -1
        velocity = self.hand.get_velocity()
        for i in range(6):
            if velocity:
                new_values.append(velocity[i])
            else:
                new_values.append(0)
        current = self.hand.get_current()
        for i in range(6):
            if current:
                new_values.append(current[i])
            else:
                new_values.append(0)

        for i in range(18):  # Update for 15 values
            self.y_data[i].append(new_values[i])

        for subplot_idx in range(3):
            for j, line in enumerate(self.lines[subplot_idx]):
                index = (
                    subplot_idx * 6 + j
                )  # Update index to select 5 values per subplot
                line.set_data(self.x_data, self.y_data[index])

        # Keep x-axis range within the last 5 seconds
        min_x = max(0, current_time - self.window_size)
        for ax in self.axes:
            ax.set_xlim(min_x, min_x + self.window_size)
            ax.relim()
            ax.autoscale_view()

        return [line for lines in self.lines for line in lines]

    def start(self):
        """Starts the real-time plotting."""
        ani = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=10,
            blit=True,
            cache_frame_data=False,
        )
        plt.show()


class RealTimePlotTouch:
    def __init__(
        self, hand: Hand, fig=None, axes=None, window_size=5, sample_rate=500
    ):
        self.hand = hand
        self.window_size = window_size
        self.sample_rate = sample_rate
        self.max_samples = (
            window_size * sample_rate
        )  # 5s * 500Hz = 2500 samples
        self.start_time = time.time()

        # Using deque for fast FIFO operations
        self.x_data = deque(maxlen=self.max_samples)
        self.y_data = [deque(maxlen=self.max_samples) for _ in range(30)]

        # Set up five subplots for 6 values each
        if fig is None:
            self.fig, self.axes = plt.subplots(
                5, 1, figsize=(12, 10), sharex=True, num="PSYONIC"
            )
        else:
            self.fig = fig
            self.axes = axes
        self.lines = []
        self.fig.suptitle("Ability Hand", fontsize=16)

        try:
            image_path = os.path.join(
                os.path.dirname(__file__), "touch_sensor_legend_sml.png"
            )
            img = mpimg.imread(image_path)
            self.fig.figimage(img, xo=30, yo=825, zorder=10)
        except FileNotFoundError:
            print("Touch sensor legend image not found.")

        y_labels = ["Index", "Middle", "Ring", "Pinky", "Thumb"]
        for i, ax in enumerate(self.axes):
            if i == 0:
                ax.set_title("Touch Sensors", fontweight="bold")
            lines = [
                ax.plot([], [], label=f"Value {j}")[0]
                for j in range(i * 6, (i + 1) * 6)
            ]
            self.lines.append(lines)
            if i == 2:
                ax.set_ylabel(
                    "Newtons (N)",
                )
            ax.set_ylim(0, 6)
            ax2 = ax.twinx()
            ax2.set_ylabel(
                y_labels[i],
                fontweight="bold",
                rotation=0,
                labelpad=30,
                ha="center",
                va="center",
            )
            ax2.set_yticks([])

        self.axes[-1].set_xlabel("Time (s)")

    def update_plot(self, frame):
        """Updates the plots with the latest data."""
        current_time = time.time() - self.start_time
        self.x_data.append(current_time)

        new_values = self.hand.get_fsr()
        for i in range(30):
            self.y_data[i].append(new_values[i])

        for subplot_idx in range(5):
            for j, line in enumerate(self.lines[subplot_idx]):
                index = (
                    subplot_idx * 6 + j
                )  # Update index to select 6 values per subplot
                line.set_data(self.x_data, self.y_data[index])

        # Keep x-axis range within the last 5 seconds
        min_x = max(0, current_time - self.window_size)
        for ax in self.axes:
            ax.set_xlim(min_x, min_x + self.window_size)
            ax.relim()
            ax.autoscale_view()

        return [line for lines in self.lines for line in lines]

    def start(self):
        """Starts the real-time plotting."""
        ani = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=10,
            blit=True,
            cache_frame_data=False,
        )
        plt.show()
