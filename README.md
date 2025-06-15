# Distributed UAV Aerial Firefighting

A MATLAB‑based simulation and control framework for a swarm of fixed‑wing UAVs that autonomously refill at depots and discharge water on dynamic wildfire fronts. The system uses a collision‑aware Voronoi–Lloyd partitioning scheme, Extended Kalman Filters for fire‑front estimation, and a consensus protocol for distributed communication.

## Features

- **Distributed Task Allocation**  
  Dynamically redistributes UAV coverage in real time via a modified Voronoi–Lloyd algorithm that enforces inter‑agent safety distances.

- **Collision Avoidance**  
  Models each UAV as a disk of radius δ; uses virtual “shifted” sites to push Voronoi bisectors outward by the combined safety distance, guaranteeing no overlap.

- **Fire‑Front Estimation**  
  Runs an Extended Kalman Filter (EKF) on noisy sensor measurements to track moving fire perimeters.

- **Consensus‑Based Sharing**  
  Implements a linear consensus algorithm for decentralized fusion of fire‑front positions and sizes across the UAV network.

- **Adaptive Refill Scheduling**  
  UAVs alternate between refill depots and active fire zones, minimizing idle time and maximizing water‑drop frequency.

- **Scalable Simulations**  
  Easily vary the number of UAVs, number and location of fires, refill station layout, and grid resolution.

## Run the code
Open MATLAB and navigate to the project folder, then:
```ruby
run project/main.m
```

This will launch a simulation with 8 UAVs, two moving fire fronts, and one refill depots.\
Inside this file there are all the important changeble parameters and their descriptions.

## Detailed description
A detailed description of the work done and all the results are described in this report: [Download the report](Distributed_System_Report.pdf)





