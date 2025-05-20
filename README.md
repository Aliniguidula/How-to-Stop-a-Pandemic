🦠 **How to Stop a Pandemic**

Simulating Disease Spread & Testing Immunization Strategies Using Graph Algorithms

📌 **Overview**

This project uses graph theory and social network analysis to simulate the spread of a pathogen in a population and evaluate strategies to slow or stop its transmission. The simulation is inspired by real-world epidemiology and explores how disease transmission pathways can be disrupted using targeted immunization techniques.

Built in Java, this program loads a population network from a dataset and models transmission based on edge weights (representing probability of infection). It implements several key epidemiological computations and simulates immunization strategies to evaluate their effectiveness.

🔬 **Topics Covered**
- Graph algorithms (adjacency lists/matrices, Dijkstra's)
- Social network modeling
- Probability-based transmission modeling
- Basic reproduction number (R₀) calculation
- Generational spread analysis
- Targeted immunization based on Node degree & Clustering coefficient

⚙️ **Features**

1. Dataset Loading:
   Reads custom or provided datasets representing undirected graphs of individuals and their contact strength (edge weights).
   Converts probabilities to integer weights for graph construction.
   Supports ignoring low-probability edges for performance and realism.

2. Transmission Path Finder
   Calculates the highest probability path between two individuals using a modified Dijkstra’s algorithm with transformed weights: weight = -log(probability)
   Returns the full transmission path if one exists.

3. Basic Reproduction Number (R₀)
   Calculates R₀ using:
   R₀ = τ * c * d
   Where:
   τ = transmissibility (input parameter)
   c = average degree of the graph
   d = duration of infectiousness (assumed to be 1)

4. Infection Generations Simulation
   Simulates how many generations (waves) it takes to infect a desired percentage of the population starting from an index case.
   Implements a breadth-first expansion algorithm.
   Handles unreachable nodes and invalid parameters gracefully.
