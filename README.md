# Path Planning Visualizer: A\* Algorithm in Python

## Overview

This project is a visual implementation of the A\* (A-star) pathfinding algorithm, designed to run on a 2D grid using the Pygame library. The goal is to provide an intuitive, interactive way to explore how the A\* algorithm works, making it a valuable learning tool and portfolio piece for robotics and AI enthusiasts.

## 🔍 What is A\*?

A\* is a best-first search algorithm that finds the shortest path between two points while avoiding obstacles. It combines features of Dijkstra's algorithm and Greedy Best-First Search using the formula:

```
f(n) = g(n) + h(n)
```

* **g(n)**: Actual cost from the start node to the current node
* **h(n)**: Heuristic estimated cost from the current node to the goal

By balancing cost and heuristic, A\* efficiently finds optimal paths in graphs and grids.

## 🎯 Project Goals

* Implement A\* from scratch in Python
* Visualize the search process using Pygame
* Master fundamental path planning concepts like g-cost, h-cost, f-cost, open/closed sets
* Create an interactive tool with user-controlled start/end nodes and obstacles
* Save the search process as an animated MP4 or GIF for showcasing

## 🗂 Repository Structure

```
path-planning-visualizer/
├── astar_visualizer.py       # Main script with visualization and algorithm
├── assets/
│   ├── example1.gif
│   ├── example2.gif
│   └── example3.gif
├── README.md
```

## 🚀 How to Run

### Prerequisites

```bash
pip install pygame imageio numpy
```

### Launch the Visualizer

```bash
python astar_visualizer.py
```

### Interactions

* **Left-click**: Place start, end, or draw barriers
* **Right-click**: Erase a node
* **Spacebar**: Start the A\* search
* **C key**: Clear the grid

## 🧠 Heuristics

The tool supports:

* **Manhattan Distance** (default)
* **Diagonal Distance** (for 8-connected movement)

The user can experiment with heuristic functions in `heuristic()` for different effects.

## 📅 Weekly Breakdown

### Week 9: Back to Fundamentals - Algorithmic Path Planning

#### **Monday: Project Kick-off & A* Theory*\*

* Created `path-planning-visualizer` repo
* Studied f(n) = g(n) + h(n) intuition
* Planned code architecture: `Node` class, grid as 2D list, Pygame chosen for visuals

#### **Tuesday: Core Implementation**

* Implemented grid and `Node` structure
* Wrote core A\* loop with open and closed sets

#### **Wednesday: Making It Visual**

* Visualized grid and states (start, end, barriers, open, closed, path)
* Integrated algorithm with Pygame's drawing loop

#### **Thursday: Interactivity & Heuristics**

* Enabled full mouse interaction
* Added diagonal movement with correct cost handling
* Switched to Octile heuristic for accurate pathfinding with diagonals

#### **Friday: Documentation & Polish**

* Added GIF/MP4 export via `imageio`
* Finalized README and code documentation

## 🎬 Example Outputs

<table>
  <tr>
    <td><strong>Example 1</strong></td>
    <td><strong>Example 2</strong></td>
    <td><strong>Example 3</strong></td>
  </tr>
  <tr>
    <td><img src="output/astar_path_20250707_140714.gif" width="250"/></td>
    <td><img src="output/astar_path_20250707_140518.gif" width="250"/></td>
    <td><img src="output/astar_path_20250707_140608.gif" width="250"/></td>
  </tr>
</table>

## 📌 Key Features

* Fully interactive grid interface
* Real-time visualization of A\* search process
* 4- or 8-connected movement support
* Save animation of each run
* Clean and modular codebase for extensions

## 🧹 Final Notes

This project is an ideal learning tool for students and practitioners interested in algorithms, AI, robotics, or game development. Feel free to fork, modify, or extend it!

---

## 🌐 Share Your Work

> "This week I took a step back from large systems to focus on a fundamental robotics algorithm: A\*. I implemented it from scratch in Python and built this interactive visualizer with Pygame. It's incredibly satisfying to watch the algorithm work its way through a maze you've drawn! Check out the GIF and the code on my GitHub: \[Link] #robotics #ai #pathplanning #python #algorithms #pygame"
