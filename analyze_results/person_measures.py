import csv
import numpy as np
import matplotlib.pyplot as plt
import os
import glob
import math
import statistics

x_range = (-1.75, 0.25)
y_range = (-1.5, 2)

test_cases = ["Walking", "Running"]
folder = '/home/autosoft/Desktop/Results/Person/'
folder = '/Users/carlhildebrandt/Dropbox/UVA/Research/Work/WorldInTheLoop/Results/Person/'

for case in test_cases:
    print("")
    print("")
    print("============================================")
    print("===================" + str(case) + "==================")
    print("============================================")
    print("")
    print("")

    simulations_files   = glob.glob(folder + case + '/Simulation/*.csv')
    mixed_reality_files = glob.glob(folder + case + '/MixedReality/*.csv')
    reality_files       = glob.glob(folder + case + '/Reality/*.csv')

    simulations_files.sort()
    mixed_reality_files.sort()
    mixed_reality_files.sort()

    # Variables used to compute averages for each of the different types of tests
    average_min = np.zeros(3)
    average_max = np.zeros(3)
    average_min_align_x = np.zeros(3)
    average_max_align_x = np.zeros(3)
    average_min_algin_y = np.zeros(3)
    average_max_algin_y = np.zeros(3)

    # Used to compute the person standard devation
    person_velocity_stdev = np.zeros(3)

    # Variables used to check if a test passed or failed
    pass_fail_filename = []
    pass_fail_list = []

    # Plot the drone and person positions
    for file_counter in range(1, 4):

        if file_counter == 1:
            files = simulations_files
        elif file_counter == 2:
            files = mixed_reality_files
        elif file_counter == 3:
            files = reality_files

        plot_counter = file_counter

        for filename in files:

            drone_position_x = []
            drone_position_y = []
            person_position_x = []
            person_position_y = []
            distance = []
            drone1person_misalignment_x = []
            drone1person_misalignment_y = []
            time_data = []
            person_velocity = [0]

            test_started = False

            with open(filename, newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    # Get the drone and person position
                    drone_x_pos = float(row["drone1_pos_x"])
                    drone_y_pos = float(row["drone1_pos_y"])
                    person_x_pos = float(row["person_pos_x"])
                    person_y_pos = float(row["person_pos_y"])
                    # Get the distance and missalignment 
                    dis = float(row["drone1person_distance"])
                    mis_x = float(row["drone1person_misallignment_x"])
                    mix_y = float(row["drone1person_misallignment_y"])
                    t = float(row["timestamp"])
                    # Test is ready when the distance is greater than 3
                    if dis > 3:
                        test_started = True
                    # Wait for the test to start
                    if test_started:
                        drone_position_x.append(drone_x_pos)
                        drone_position_y.append(drone_y_pos)
                        person_position_x.append(person_x_pos)
                        person_position_y.append(person_y_pos)
                        distance.append(dis)
                        drone1person_misalignment_x.append(mis_x)
                        drone1person_misalignment_y.append(mix_y)
                        time_data.append(t)
        

            # Compute the persons velocity
            start_time = time_data[0]
            start_position = (person_position_x[0], person_position_y[0])
            for i in range(0, len(time_data) - 1):
                current_time = time_data[i + 1]
                dt = current_time - start_time
                start_time = current_time

                current_position = (person_position_x[i + 1], person_position_y[i + 1])
                dx = current_position[0] - start_position[0]
                dy = current_position[1] - start_position[1]
                start_position = current_position

                distance_traveled = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
                velocity = distance_traveled / dt
                person_velocity.append(velocity)

            if case == "Walking":
                plt.figure(1)
            else:
                plt.figure(2)

            plt.subplot(5, 3, plot_counter)
            plt.plot(drone_position_x, drone_position_y, color='C0')
            plt.plot(person_position_x, person_position_y, color='C1')
            plt.xlabel("X Position")
            plt.ylabel("Y Position")
            if file_counter == 1:
                plt.title("Simulation")
            elif file_counter == 2:
                plt.title("Mixed Reality")
            elif file_counter == 3:
                plt.title("Reality")
            plt.xlim([-2.5, 3])
            plt.ylim([-1.5, 2.5])

            if case == "Walking":
                plt.figure(3)
            else:
                plt.figure(4)

            plt.subplot(5, 3, plot_counter)
            plt.plot(time_data, person_velocity, color='C0')
            plt.xlabel("Time (s)")
            plt.ylabel("Velocity (m/s_")
            if file_counter == 1:
                plt.title("Simulation")
            elif file_counter == 2:
                plt.title("Mixed Reality")
            elif file_counter == 3:
                plt.title("Reality")

            plot_counter += 3

            # Print the min and max values for each of the allginments
            print(filename)
            print("Distance Range: [" + str(min(distance)) + ", " + str(max(distance)) + "]")
            print("Position Range X: [" + str(min(drone_position_x)) + ", " + str(max(drone_position_x)) + "]")
            print("Position Range Y: [" + str(min(drone_position_y)) + ", " + str(max(drone_position_y)) + "]")
            print("Misalignment X: [" + str(min(drone1person_misalignment_x)) + ", " + str(max(drone1person_misalignment_x)) + "]")
            print("Misalignment Y: [" + str(min(drone1person_misalignment_y)) + ", " + str(max(drone1person_misalignment_y)) + "]")

            # Compute averages for all tests
            average_min[file_counter - 1] += min(distance)
            average_max[file_counter - 1] += max(distance)
            average_min_align_x[file_counter - 1] += min(drone1person_misalignment_x)
            average_max_align_x[file_counter - 1] += max(drone1person_misalignment_x)
            average_min_algin_y[file_counter - 1] += min(drone1person_misalignment_y)
            average_max_algin_y[file_counter - 1] += max(drone1person_misalignment_y)

            # Computer person standard deviation
            person_velocity_stdev[file_counter - 1] += statistics.stdev(person_velocity) 

            # Check if it passed or failed
            pass_fail_filename.append(filename)
            failed = False
            if (x_range[0] > min(drone_position_x)) or (x_range[1] < max(drone_position_x)):
                failed = True
                print("Failed: " + str(x_range[0]) + ">" + str(min(drone_position_x)) + " or " + str(x_range[1]) + "<" + str(max(drone_position_x)))
            if (y_range[0] > min(drone_position_y)) or (y_range[1] < max(drone_position_y)):
                failed = True
                print("Failed: " + str(y_range[0]) + ">" + str(min(drone_position_y)) + " or " + str(y_range[1]) + "<" + str(max(drone_position_y)))
            pass_fail_list.append(failed)

            print("--------------------------------")

    print("============================================")
    print("===================SUMMARY==================")
    print("============================================")

    # Display averages for each
    for i in range(0, 3): 
        average_min[i] = average_min[i] / 5.0
        average_max[i] = average_max[i] / 5.0
        average_min_align_x[i] = average_min_align_x[i] / 5.0
        average_max_align_x[i] = average_max_align_x[i] / 5.0
        average_min_algin_y[i] = average_min_algin_y[i] / 5.0
        average_max_algin_y[i] = average_max_algin_y[i] / 5.0

        person_velocity_stdev[i] = person_velocity_stdev[i] / 5.0

        if i == 0:
            print("Simulation")
        elif i == 1:
            print("Mixed Reality")
        elif i == 2:
            print("Reality") 
        print("Distance Range: [" + str(average_min[i]) + ", " + str(average_max[i]) + "]")
        print("Misalignment X: [" + str(average_min_align_x[i]) + ", " + str(average_max_align_x[i]) + "]")
        print("Misalignment Y: [" + str(average_min_algin_y[i]) + ", " + str(average_max_algin_y[i]) + "]")
        print("Person Velocity Standard Deviation: [" + str(average_max_algin_y[i]) + "]")
        print("--------------------------------")


    print("============================================")
    print("===============TEST PASS FAIL===============")
    print("============================================")


    print("Requirements")
    print("X Range: " + str(x_range))
    print("Y Range: " + str(y_range))
    print("--------------------------------")

    pass_counter = np.zeros(3, dtype="int")
    fail_counter = np.zeros(3, dtype="int")

    for j in range(0, len(pass_fail_filename)):

        # Check what scenario we are in
        if "_Simulation" in pass_fail_filename[j]:
            counter_index = 0
        if "_MixedReality" in pass_fail_filename[j]:
            counter_index = 1
        if "_Reality" in pass_fail_filename[j]:
            counter_index = 2

        # Pass Fail Decision
        pass_fail_string = ""
        if pass_fail_list[j] == True:
            pass_fail_string = "Failed"
            fail_counter[counter_index] += 1
        else:
            pass_fail_string = "Passed"
            pass_counter[counter_index] += 1

        print(pass_fail_filename[j] + ": " + pass_fail_string)


    # Display summary of events:
    print("--------------------------------")
    print("Simulation : pass - " + str(pass_counter[0]) + " fail - " + str(fail_counter[0]))
    print("Mixed Reality : pass - " + str(pass_counter[1]) + " fail - " + str(fail_counter[1]))
    print("Reality : pass - " + str(pass_counter[2]) + " fail - " + str(fail_counter[2]))


plt.show()
