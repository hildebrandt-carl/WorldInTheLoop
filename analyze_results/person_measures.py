import csv
import numpy as np
import matplotlib.pyplot as plt
import os
import glob

required_distance = (2, 4.5)
required_misalign_x = (-10, 10) 
required_misalign_y = (-2.5, 2.5)
required_overshoot_y = (-1, 2)

test_cases = ["Walking", "Running"]

for case in test_cases:

    simulations_files   = glob.glob('/home/autosoft/Desktop/Results/Person/' + case + '/Simulation/*.csv')
    mixed_reality_files = glob.glob('/home/autosoft/Desktop/Results/Person/' + case + '/MixedReality/*.csv')
    reality_files       = glob.glob('/home/autosoft/Desktop/Results/Person/' + case + '/Reality/*.csv')

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
            drone1person_misallignment_x = []
            drone1person_misallignment_y = []

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
                        drone1person_misallignment_x.append(mis_x)
                        drone1person_misallignment_y.append(mix_y)
        
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

            plot_counter += 3

            # Print the min and max values for each of the allginments
            print(filename)
            print("Distance Range: [" + str(min(distance)) + ", " + str(max(distance)) + "]")
            print("Misalignment X: [" + str(min(drone1person_misallignment_x)) + ", " + str(max(drone1person_misallignment_x)) + "]")
            print("Misalignment Y: [" + str(min(drone1person_misallignment_y)) + ", " + str(max(drone1person_misallignment_y)) + "]")
            print("--------------------------------")

            average_min[file_counter - 1] += min(distance)
            average_max[file_counter - 1] += max(distance)
            average_min_align_x[file_counter - 1] += min(drone1person_misallignment_x)
            average_max_align_x[file_counter - 1] += max(drone1person_misallignment_x)
            average_min_algin_y[file_counter - 1] += min(drone1person_misallignment_y)
            average_max_algin_y[file_counter - 1] += max(drone1person_misallignment_y)

            # Check if it passed or failed
            pass_fail_filename.append(filename)
            failed = False
            if (required_distance[0] > min(distance)) or (required_distance[1] < max(distance)):
                failed = True
            if (required_misalign_x[0] > min(drone1person_misallignment_x)) or (required_misalign_x[1] < max(drone1person_misallignment_x)):
                failed = True
            if (required_misalign_y[0] > min(drone1person_misallignment_y)) or (required_misalign_y[1] < max(drone1person_misallignment_y)):
                failed = True
            if (required_overshoot_y[0] > min(drone_position_y)) or (required_overshoot_y[1] < max(drone_position_y)):
                failed = True
            pass_fail_list.append(failed)

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

        if i == 0:
            print("Simulation")
        elif i == 1:
            print("Mixed Reality")
        elif i == 2:
            print("Reality") 
        print("Distance Range: [" + str(average_min[i]) + ", " + str(average_max[i]) + "]")
        print("Misalignment X: [" + str(average_min_align_x[i]) + ", " + str(average_max_align_x[i]) + "]")
        print("Misalignment Y: [" + str(average_min_algin_y[i]) + ", " + str(average_max_algin_y[i]) + "]")
        print("--------------------------------")


    print("============================================")
    print("===============TEST PASS FAIL===============")
    print("============================================")


    print("Requirements")
    print("Distance requirement: " + str(required_distance))
    print("Allignment X requirement: " + str(required_misalign_x))
    print("Allignment Y requirement: " + str(required_misalign_y))
    print("--------------------------------")

    for j in range(0, len(pass_fail_filename)):
        pass_fail_string = "Passed"
        if pass_fail_list[j] == True:
            pass_fail_string = "Failed"
        print(pass_fail_filename[j] + ": " + pass_fail_string)

plt.show()
