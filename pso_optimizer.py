import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from leg_simulate import run_simulation, init_simulation, unload_simulation

# Define the PSO parameters
class Particle:
    def __init__(self) -> None:
        # Initialize a particle with random weights and velocities
        self.position = np.random.uniform([poisson_min, youngs_min], [poisson_max, youngs_max])
        self.velocity = np.random.uniform([-0.2, -500], [0.2, 500])
        self.best_position = np.copy(self.position)
        self.best_score = float('inf')

def objective_function(particle: np.ndarray, sample_size: int =250) -> float:
    """
    Calculate the portfolio's performance.
    - weights: Asset weights in the portfolio.
    - returns: Expected returns of the assets.
    - covariance: Covariance matrix representing risk.
    """
    print("Particle:", particle)
    poisson_ratio = particle[0]
    youngs_modulus = particle[1]
    init_simulation(youngs_modulus, poisson_ratio)
    rms_error = 0
    for n in range(sample_size):
        i = np.random.randint(0, num_rows)
        sim_point = run_simulation(inputs[i])
        # print("Simulated Point:", sim_point)
        # print("Actual Point:", actual_output[i])
        rms_error += calculate_rmse(sim_point, actual_output[i])
    rms_error /= sample_size
    unload_simulation()
    
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

def update_particles(particles: Particle, global_best_position: np.ndarray, w: float, c1: float, c2: float) -> None:
    """
    Update the position and velocity of each particle.
    - particles: List of particle objects.
    - global_best_position: Best position found by all particles.
    - returns: Expected returns of the assets.
    - covariance: Covariance matrix representing risk.
    - w: Inertia weight to control particle's previous velocity effect.
    - c1: Cognitive coefficient to pull particles towards their own best position.
    - c2: Social coefficient to pull particles towards the global best position.
    """
    for particle in particles:
        # Random coefficients for velocity update
        r1, r2 = np.random.rand(len(particle.position)), np.random.rand(len(particle.position))
        # Update velocity
        particle.velocity = (w * particle.velocity +
                             c1 * r1 * (particle.best_position - particle.position) +
                             c2 * r2 * (global_best_position - particle.position))
        # Update position
        particle.position += particle.velocity
        particle.position = np.clip(particle.position, [poisson_min, youngs_min], [poisson_max, youngs_max])  # Ensure weights are between 0 and 1
        
def pso_portfolio_optimization(n_particles: int, n_iterations: int, sample_size: int = 500) -> tuple:
    """
    Perform Particle Swarm Optimization to find the optimal asset weights.
    - n_particles: Number of particles in the swarm.
    - n_iterations: Number of iterations for the optimization.
    - returns: Expected returns of the assets.
    - covariance: Covariance matrix representing risk.
    """
    # Initialize particles
    particles = [Particle() for _ in range(n_particles)]
    # Initialize global best position
    global_best_position = Particle().position
    global_best_score = float('inf')

    with open('particle_swarm.log', 'w') as f:
        f.write("Particle Swarm Optimization\n")
        f.write("---------------------------\n")
        f.write("Number of Particles: " + str(n_particles) + "\n")
        f.write("Number of Iterations: " + str(n_iterations) + "\n")
        f.write("sample_size: " + str(sample_size) + "\n")
        f.write("----------------------------------------------\n")
        # PSO parameters
        w = 0.8  # Inertia weight: how much particles are influenced by their own direction
        c1 = 1.8  # Cognitive coefficient: how well particles learn from their own best solutions
        c2 = 0.5  # Social coefficient: how well particles learn from global best solutions
        history = []  # To store the best score at each iteration
        
        for _ in range(n_iterations):
            update_particles(particles, global_best_position, w, c1, c2)
            for particle in particles:
                score = objective_function(particle.position, sample_size=sample_size)
                if score < global_best_score:
                    # Update the global best position and score
                    global_best_position = np.copy(particle.position)
                    global_best_score = score
                if score < particle.best_score:
                    # Update the particle's best position and score
                    particle.best_position = np.copy(particle.position)
                    particle.best_score = score
            f.write("Iteration: " + str(_) + "\t")
            f.write("Best Particle: " + str(global_best_position) + "\t")
            f.write("Global Best Score: " + str(global_best_score) + "\n")
            # Store the best score (negative return/risk ratio) for plotting
            history.append(global_best_score)
        
        f.write("=============================================\n")
        f.write("Optimal Parameters: " + str(global_best_position) + "\t")
        f.write("Score: " + str(global_best_score) + "\n")

        f.close()

        return global_best_position, history

# Run the PSO algorithm
n_particles = 50  # Number of particles
n_iterations = 15  # Number of iterations
poisson_min = 0.01
poisson_max = 0.499
youngs_min = 0
youngs_max = 25000

actual_dataset = pd.read_csv('real_leg_xyz.csv')

inputs = actual_dataset.iloc[:, 0:3]  # Selects all rows and the first three columns (l1, l2, l3)
inputs = inputs.to_numpy()  # Selects all rows and the first three columns (l1, l2, l3)

actual_output = actual_dataset.iloc[:, 3:6]  # Selects all rows and the last three columns (x, y, z)
actual_output = actual_output.to_numpy()  # Selects all rows and the last three columns (x, y, z)

# print("Inputs (l1, l2, l3):\n", inputs)
# print("Outputs (x, y, z):\n", actual_output)

num_rows = len(inputs)

best_weights, optimization_history = pso_portfolio_optimization(n_particles, n_iterations)

# Plotting the optimization process
plt.figure(figsize=(12, 6))
plt.plot(optimization_history, marker='o')
plt.title('Leg Parameters Optimization Using Particle Swarm Optimization')
plt.xlabel('Iteration')
plt.ylabel('Objective Function Value (RMS Error)')
plt.grid(False)  # Turn off gridlines
plt.show()

# Display the optimal asset weights
print(f"Optimal Asset Weights: {best_weights}")