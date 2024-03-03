import numpy as np
import json
import os
import matplotlib.pyplot as plt


log_name = "group_exploration2024-02-27_19-07-49.json"
log_file = f"/home/leev/simulation_logs/{log_name}"

log_name2 = "modified_pso2024-02-28_11-22-01.json"
log_file2 = f"/home/leev/simulation_logs/{log_name2}"

log_name3 = "frontier_pso2024-02-28_11-34-57.json"
log_file3 = f"/home/leev/simulation_logs/{log_name3}"


def main():
    loaded_dict = None
    with open(f"{log_file}", "r+") as log_fh:
        loaded_dict = json.load(log_fh)

    loaded_dict2 = None
    with open(f"{log_file2}", "r+") as log_fh:
        loaded_dict2 = json.load(log_fh)

    loaded_dict3 = None
    with open(f"{log_file3}", "r+") as log_fh:
        loaded_dict3 = json.load(log_fh)

    percentage_arr = []
    simulation_seconds_arr = []
    average_covered_path_arr = []
    
    for el in loaded_dict['data']:
        simulation_seconds_arr.append(el['simulation_seconds'])
        percentage_arr.append(el['percentage'])
        average_covered_path_arr.append(el['average_covered_path'])
        
    percentage = np.array(percentage_arr)
    simulation_seconds = np.array(simulation_seconds_arr)
    average_covered_path = np.array(average_covered_path_arr)

    percentage_arr2 = []
    simulation_seconds_arr2 = []
    average_covered_path_arr2 = []
    
    for el in loaded_dict2['data']:
        simulation_seconds_arr2.append(el['simulation_seconds'])
        percentage_arr2.append(el['percentage'])
        average_covered_path_arr2.append(el['average_covered_path'])
        
    percentage2 = np.array(percentage_arr2)
    simulation_seconds2 = np.array(simulation_seconds_arr2)
    average_covered_path2 = np.array(average_covered_path_arr2)

    percentage_arr3 = []
    simulation_seconds_arr3 = []
    average_covered_path_arr3 = []
    
    for el in loaded_dict3['data']:
        simulation_seconds_arr3.append(el['simulation_seconds'])
        percentage_arr3.append(el['percentage'])
        average_covered_path_arr3.append(el['average_covered_path'])
        
    percentage3 = np.array(percentage_arr3)
    simulation_seconds3 = np.array(simulation_seconds_arr3)
    average_covered_path3 = np.array(average_covered_path_arr3)
    
    
    plt.plot(simulation_seconds, percentage)
    plt.plot(simulation_seconds2, percentage2)
    plt.plot(simulation_seconds3, percentage3)
    plt.show()
    
    plt.plot(simulation_seconds, average_covered_path)
    plt.plot(simulation_seconds2, average_covered_path2)
    plt.plot(simulation_seconds3, average_covered_path3)
    plt.show()
    
    
main()