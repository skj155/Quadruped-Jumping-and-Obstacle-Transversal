Project: Quadruped Jumping and Obstacle Traversal 
Mentor: Prof. Dipayan Mukherjee 
Objective: • Develop 6 DOF MPC-driven quadruped framework for agile jumping and 0.05m stair-climbing stabilization

Approach:
• Derived 6 DOF rigid-body dynamics via Newton-Euler, deploying a 15 step convex MPC and QP solver
• Optimized 4 jump phases at 100Hz, and simulated 10.0kg torso’s 5-second zero-torque angular momentum

Impact:
• Cleared 40cm obstacles under 9.81m/s², achieving 90% accuracy, 15% pitch, and 0 drift 6-DoF recovery
• Shifted dynamic leg-exchange thresholds to 0.3m, accumulating sufficient orbital energy to climb 0.05m stairs
