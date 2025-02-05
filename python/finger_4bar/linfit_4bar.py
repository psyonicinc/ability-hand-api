from abh_finger_4bar import *
import matplotlib.pyplot as plt
from numpy.polynomial import Polynomial

# get true relationship
q1 = np.linspace(0, 120, 1000) * np.pi / 180
q2 = q1 * 0.0
for i in range(0, q1.size):
    q2[i] = get_abh_4bar_driven_angle(q1[i])

# compute best fit line
c = np.polyfit(q1, q2, 1)
print("bestfit coefficients: ", c)
linfit_q2 = q1 * c[0] + c[1]

# Plot the true value against the best fit line
fig, axs = plt.subplots(1, 1)
(line1,) = axs.plot(q1 * 180 / np.pi, q2 * 180 / np.pi)
(line2,) = axs.plot(q1 * 180 / np.pi, linfit_q2 * 180 / np.pi)
axs.set_ylabel("q2 (deg)")
axs.set_xlabel("q1 (deg)")
axs.set_title("Ability Hand Finger 4bar Relationship Across Full Range of Motion")
axs.legend([line1, line2], ["True Value", "Linear Approximation"])

# Plot the error function
error = q2 - linfit_q2
error_deg = error * 180 / np.pi
print("max error (deg):", np.max(np.abs(error_deg)))
fig2, axs2 = plt.subplots(1, 1)
(line3,) = axs2.plot(q1 * 180 / np.pi, error_deg)
axs2.set_title("Error: Best Fit Line to True Value")
axs2.set_ylabel("Error (deg)")
axs2.set_xlabel("q1 (deg)")

fig.tight_layout()
plt.show()
