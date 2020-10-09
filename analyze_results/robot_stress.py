import csv
import numpy as np
import matplotlib.pyplot as plt
import os
import glob

result = glob.glob('FirstFullRun/*.csv')
result.sort()


plot_counter = 0

for filename in result:
    plot_counter += 1
    force_list = []
    time_list = []
    start_time = -1
    with open(filename, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            if float(row["timestamp"]) != 0:
                force_list.append(float(row["total force"]))
                if start_time == -1:
                    start_time = float(row["timestamp"])
                time = float(row["timestamp"]) - start_time
                time_list.append(time)

    plt.figure(1)
    plt.subplot(4, 4, plot_counter)
    plt.plot(time_list, force_list)
    plt.title(filename)
    plt.xlabel("Time (S)")
    plt.ylabel("Force (N)")
    plt.ylim([0, 1])

    # Compute the highest force
    largest_force = max(force_list)

    scatter_x = 0
    color = 'red'

    if "avoidance" in filename.lower():
        scatter_x = 1
    elif "gate" in filename.lower():
        scatter_x = 2
    elif "person" in filename.lower():
        scatter_x = 3

    if "simulation" in filename.lower():
        color = 'blue'
    elif "mrunitysphinx" in filename.lower():
        color = 'green'
    elif "mrunityreal" in filename.lower():
        color = 'black'

    plt.figure(2)
    plt.scatter(scatter_x, largest_force, color=color)


plt.show()

# Compute the highest point