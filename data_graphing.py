import matplotlib.pyplot as plt

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

gen_min = min(genetic)
pso_min = min(pso)

plt.figure(figsize=(12, 6))
plt.plot(genetic, label='Genetic Algorithm', color='red', linestyle='-')
plt.plot(pso, label='Particle Swarm Optimization', color='blue', linestyle='-.')
plt.scatter(genetic.index(gen_min), gen_min, color='red', marker='o', label='Genetic Algorithm Min')
plt.scatter(pso.index(pso_min), pso_min, color='blue', marker='o', label='Particle Swarm Optimization Min')
plt.xlabel('Iteration/Generation')
plt.ylabel('Root Mean Squared Error')
plt.legend()
plt.show()