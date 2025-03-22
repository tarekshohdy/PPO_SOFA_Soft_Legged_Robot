import pandas as pd
import numpy as np
from leg_simulate import run_simulation, init_simulation, unload_simulation


actual_dataset = pd.read_csv('real_leg_xyz.csv')

inputs = actual_dataset.iloc[:, 0:3]  # Selects all rows and the first three columns (l1, l2, l3)
inputs = inputs.to_numpy()  # Selects all rows and the first three columns (l1, l2, l3)

actual_output = actual_dataset.iloc[:, 3:6]  # Selects all rows and the last three columns (x, y, z)
actual_output = actual_output.to_numpy()  # Selects all rows and the last three columns (x, y, z)

num_rows = len(inputs)

def fitness_function(individual: tuple) -> float:
    '''
    Calculate the fitness of an individual based on the Root Mean Squared Error (RMSE) between the simulated and actual points.

    Parameters:
        individual (tuple): Tuple with the Young's modulus and Poisson's ratio

    Returns:
        float: RMSE value
    '''
    poisson_ratio, youngs_modulus = individual
    init_simulation(youngs_modulus, poisson_ratio)
    rms_error = 0
    for i in range(num_rows):
        sim_point = run_simulation(inputs[i])
        rms_error += calculate_rmse(sim_point, actual_output[i])
    rms_error /= num_rows
    unload_simulation()
    print("individual: ", individual, "     fitness: ", rms_error)
    return rms_error

def calculate_rmse(points_a: np.array, points_b) -> float:
    '''
    Calculate the Root Mean Squared Error (RMSE) between two sets of points.

    Parameters:
        points_a (np.array): Array with the first set of points
        points_b (np.array): Array with the second set of points

    Returns:
        float: RMSE value
    '''
    
    if points_a.shape != points_b.shape:
        raise ValueError("Both lists must have the same shape and each point must have three coordinates (X, Y, Z)")
    
    squared_errors = np.square(points_a - points_b)
    mean_squared_errors = np.mean(np.sum(squared_errors))
    rmse_value = np.sqrt(mean_squared_errors)
    return rmse_value

def data_import(file_path: str) -> tuple:
    '''
    Import data of best individuals and fitness values from a file.

    Parameters:
        file_path (str): Path to the file

    Returns:
        tuple: Tuple with the fitness values and the best individuals

    '''
    data = []
    fitness_values = []
    best_values = []
    poisson = 0
    youngs = 0
    with open(file_path, 'r') as f:
        for line in f:
            data.append(line.strip())
    for _ in range(6):
        data.pop(0)
    best_values = data.pop(-1)
    data.pop(-1)
    best_values = best_values.split('\t')
    best_values = best_values[0].split(' ')
    for i in range(len(best_values)):
        if len(best_values[i]) == 0:
            continue
        if best_values[i][0] == '[':
            poisson = float(best_values[i][1:])
        if best_values[i][-1] == ']':
            best_values[i].replace(']', '')
            youngs = float(best_values[i][:-1])
    for i in range(len(data)):
        data[i] = data[i].split('\t')
        for j in range(len(data[i])):
            data[i][j] = data[i][j].split(' ')
            if data[i][j][0] == 'Fitness:' or data[i][j][0] == 'Global':
                fitness_values.append(float(data[i][j][-1]))
    return fitness_values, (poisson, youngs)

genetic, gen_best = data_import('genetic.log')
pso, pso_best = data_import('particle_swarm.log')

gen_rmse = fitness_function(gen_best)
pso_rmse = fitness_function(pso_best)

print("Genetic Algorithm RMSE: ", gen_rmse) #individual:  (0.1742459030014992, 17381.496940515673)      fitness:  16.473918397753092
print("Particle Swarm Optimization RMSE: ", pso_rmse) #individual:  (0.0821706941, 15997.6155)      fitness:  16.4248457275142