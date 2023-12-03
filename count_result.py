import csv
import numpy as np

# Initialize the sum of scores
total_score = 0
total_score_min = 0

# Define the path to your CSV file
csv_file = "results.csv"
csv_file_min = "instances/min-sum-of-cost.csv"
# Read the CSV file

score_cbs_ar = np.array([])
score_min_ar = np.array([])

with open(csv_file, "r") as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        # Ensure each row has at least two columns
        if len(row) >= 2:
            try:
                score = int(row[-1])
                score_cbs_ar = np.append(score_cbs_ar, score)
                total_score += score  # Add the score to the total
            except ValueError:
                print(f"Error parsing score in row: {row}")

with open(csv_file_min, "r") as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        # Ensure each row has at least two columns
        if len(row) >= 2:
            try:
                score = int(row[1])
                score_min_ar = np.append(score_min_ar, score)
                total_score_min += score  # Add the score to the total
            except ValueError:
                print(f"Error parsing score in row: {row}")

dif_ar = score_cbs_ar - score_min_ar
# Print the scores
print(f"Total Score: {total_score}")
print(f"Total Score optimal: {total_score_min}")
print(f"Score CBS: {score_cbs_ar}")
print(f"Score Min: {score_min_ar}")
print(f"Difference CBS - optimal: {dif_ar}")

print_lst = []
for index in range(len(dif_ar)):
    if dif_ar[index] != 0:
        print_lst.append(str('Test: ' + str(index)))
        print_lst.append(dif_ar[index])

print(print_lst)
#1923 True, random, False (1)
#1889 True, random, False (2)
#1916 True, random, False (3)
#1850 optimal
#1885 True, lower, higher False (Always)
#1925 Random
#1937 False
#1899 True, random false: lower priority
#Test 41 is weird
#Distributed 1911
# Distributed optimal 1847
# CBS optimal 1845