import numpy as np
import json
import os
import matplotlib.pyplot as plt


log_name = "group_exploration2024-02-27_19-07-49.json"
log_file = f"/home/leev/simulation_logs/{log_name}"

log_name2 = "modified_pso2024-02-27_19-52-58.json"
log_file2 = f"/home/leev/simulation_logs/{log_name2}"

def main():
    loaded_dict = None
    with open(f"{log_file}", "r+") as log_fh:
        loaded_dict = json.load(log_fh)

    loaded_dict2 = None
    with open(f"{log_file2}", "r+") as log_fh:
        loaded_dict2 = json.load(log_fh)

    percentage_arr = []
    simulation_seconds_arr = []
    
    for el in loaded_dict['data']:
        simulation_seconds_arr.append(el['simulation_seconds'])
        percentage_arr.append(el['percentage'])
        
    percentage = np.array(percentage_arr)
    simulation_seconds = np.array(simulation_seconds_arr)
    

    percentage_arr2 = []
    simulation_seconds_arr2 = []
    
    for el in loaded_dict2['data']:
        simulation_seconds_arr2.append(el['simulation_seconds'])
        percentage_arr2.append(el['percentage'])
        
    percentage2 = np.array(percentage_arr2)
    simulation_seconds2 = np.array(simulation_seconds_arr2)
    
    plt.plot(simulation_seconds, percentage)
    plt.plot(simulation_seconds2, percentage2)
    plt.show()

main()