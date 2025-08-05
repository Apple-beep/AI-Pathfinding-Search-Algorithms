# 🧠 AI Pathfinding with Greedy and A* Search

This project was created as part of the **CS480: Introduction to Artificial Intelligence** course.  
It implements two popular heuristic search algorithms — **Greedy Best-First Search** and **A\*** — to solve a map-based pathfinding problem using CSV inputs.

> Built using Python, this project parses driving and straight-line distance data between cities/states to compute efficient paths from a given start to a goal.

---

## 🚀 Features

- Implements **Greedy Best-First Search** and **A\*** search
- Reads input from CSV-based **driving distance** and **heuristic (straight-line)** maps
- Outputs:
  - Optimal path found
  - Total cost
  - Number of expanded nodes
  - Execution time
- Modular design using `SearchNode` and `SearchProblem` classes
- Robust CLI input validation

---

## 📁 Project Structure

.
├── cs480_P01_A20501777.py # Main program
├── driving_.csv # Driving distances between cities
├── straightline.csv # Heuristic distances (straight-line) between cities
└── README.md # Project documentation
---

## 💻 How to Run

### ⚙️ Requirements

- Python 3.x

### 🧾 Run the program

```bash
python cs480_P01_A20501777.py START_STATE GOAL_STATE
