import numpy as np
import pandas as pd
from deap import base, creator, tools, algorithms 
from leg_simulate import run_simulation, init_simulation, unload_simulation

# Load the CSV file
actual_dataset = pd.read_csv('real_leg_xyz.csv')

inputs = actual_dataset.iloc[:, 0:3]  # Selects all rows and the first three columns (l1, l2, l3)
inputs = inputs.to_numpy()  # Selects all rows and the first three columns (l1, l2, l3)

actual_output = actual_dataset.iloc[:, 3:6]  # Selects all rows and the last three columns (x, y, z)
actual_output = actual_output.to_numpy()  # Selects all rows and the last three columns (x, y, z)

# print("Inputs (l1, l2, l3):\n", inputs)
# print("Outputs (x, y, z):\n", actual_output)

num_rows = len(inputs)
# print("Number of rows:", num_rows)

poisson_min = 0.01
poisson_max = 0.499
youngs_min = 0
youngs_max = 25000
num_generations = 15
mutation_rate = 0.3
population_size = 50

def fitness_function(individual: tuple, sample_size: int = 500) -> float:
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
    for n in range(sample_size):
        i = np.random.randint(0, num_rows)
        sim_point = run_simulation(inputs[i])
        rms_error += calculate_rmse(sim_point, actual_output[i])
    rms_error /= sample_size
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

# Crossover function
def crossover(parent1: np.array, parent2: np.array) -> tuple:
    '''
    Perform a crossover operation between two parents to generate two children.

    Parameters:
        parent1 (np.array): Array with the first individual
        parent2 (np.array): Array with the second individual

    Returns:
        tuple: Tuple with the two children
    '''
    alpha = np.random.uniform(0, 1)
    child1 = np.array([alpha * p1 + (1 - alpha) * p2 for p1, p2 in zip(parent1, parent2)])
    child2 = np.array([alpha * p2 + (1 - alpha) * p1 for p1, p2 in zip(parent1, parent2)])
    return child1, child2

def selection(population: np.array, fitnesses: np.array, tournament_size: int =3)-> list:
    '''
    Select the best individuals from the population using a tournament selection process.

    Parameters:
        population (np.array): Array with the population
        fitnesses (np.array): Array with the fitness values of the population
        tournament_size (int): Number of individuals in each tournament

    Returns:
        list: List with the selected individuals
    '''
    selected = []
    pop_fit = list(zip(population, fitnesses))
    for _ in range(len(population)):
        ind = np.random.randint(0, len(population), tournament_size)
        tournament = [pop_fit[i] for i in ind]
        winner = min(tournament, key=lambda x: x[1])[0]
        selected.append(winner)
    return selected

def mutation(individual: np.array, mutation_rate: float =0.3) -> np.array:
    '''
    Perform a mutation operation on an individual.

    Parameters:
        individual (np.array): Array with the individual
        mutation_rate (float): Probability of mutation for each gene

    Returns:
        np.array: Array with the mutated individual
    '''
    mutated = individual
    if np.random.uniform(0, 1) < mutation_rate:
        mutated[0] = individual[0] + np.random.uniform(-poisson_max/2, poisson_max/2)
        mutated[0] = np.clip(mutated[0], poisson_min, poisson_max)
    if np.random.uniform(0, 1) < mutation_rate:
        mutated[1] = individual[1] + np.random.uniform(-youngs_max/10, youngs_max/10)
        mutated[1] = np.clip(mutated[1], youngs_min, youngs_max)
    return mutated

def genetic() -> None:
    '''
    Run the genetic algorithm to optimize the parameters of the leg model.
    '''
    population = np.random.uniform([poisson_min, youngs_min], [poisson_max, youngs_max], (population_size, 2))
    best_individuals = []
    sample_size = 500

    with open('genetic.log', 'w') as f:
        f.write('Genetic Algorithm\n')
        f.write('-----------------\n')
        f.write('Population size: ' + str(population_size) + '\n')
        f.write('Number of generations: ' + str(num_generations) + '\n')
        f.write("sample_size: " + str(sample_size) + "\n")
        f.write('----------------------------------------------\n')
        
        for g in range(num_generations):
            print("Generation:", g)
            f.write('Generation: ' + str(g))
            f.write('\t')
            fitness_values = np.array([fitness_function(individual, sample_size=sample_size) for individual in population])
            best_individual = population[np.argmin(fitness_values)]
            best_individuals.append((list(best_individual), np.min(fitness_values))) # Store the best individual of each generation
            f.write('Best individual: ' + str(best_individual))
            f.write('\t')
            f.write('Fitness: ' + str(np.min(fitness_values)))
            selected = selection(population, fitness_values)
            offspring = []

            for i in range(0, len(selected), 2):
                parent1 = selected[i]
                parent2 = selected[i+1]
                child1, child2 = crossover(parent1, parent2)
                offspring.append(mutation(child1, mutation_rate))
                offspring.append(mutation(child2, mutation_rate))
            
            population = np.array(offspring)
            population[0] = best_individual
            f.write('\n')
        
        print("Best individuals:", best_individuals)
        print("=============================================")
        best = min(best_individuals, key=lambda x: x[1])
        print("Best individual:", best)
        f.write('=============================================')
        f.write('\n')
        f.write('Overall Best individual: ' + str(best[0]))
        f.write('\t')
        f.write('Fitness: ' + str(best[1]))
        f.close()
genetic()