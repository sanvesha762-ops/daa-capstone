# Mini-Courier Planner

A Python prototype for parcel prioritization, single-van load optimization, and delivery route planning built as the capstone project for **Design & Analysis of Algorithms (ETCCDA302)**.

- **Author:** Anvesha Saxena
- **University:** K.R. Mangalam University, Gurugram
- **Course:** Design & Analysis of Algorithms (ETCCDA302)
- **Faculty:** Ankita Gupta
- **Repository:** [https://github.com/sanvesha762-ops/daa-capstone.git](https://github.com/sanvesha762-ops/daa-capstone.git)

---

## Problem Statement

Given a set of customer parcels (each with a destination, weight, priority score, and delivery deadline) and an undirected weighted road network anchored at a central depot (Node `0`), the planner solves three operational sub-problems for a single delivery van with a **25 kg** capacity:
1. Rank parcels by delivery urgency.
2. Select the subset of parcels that maximizes total priority without exceeding the van's weight limit.
3. Compute a shortest-distance round-trip delivery route starting and ending at the depot.

---

## Algorithms & Complexity

| Pipeline Stage | Algorithm Used | Rationale | Time Complexity |
| :--- | :--- | :--- | :--- |
| **1. Parcel Prioritization** | **Merge Sort** | Guaranteed $O(n \log n)$ worst-case runtime and stable sorting so ties preserve relative order | $O(n \log n)$ |
| **2. Load Selection** | **0/1 Knapsack (DP)** | Parcels cannot be split; Dynamic Programming guarantees the exact maximum-priority load | $O(n \times W)$ |
| **3. Shortest Road Paths** | **Dijkstra's Algorithm** (`heapq`) | All road distances are non-negative; computes exact shortest paths between all required stops | $O((V + E) \log V)$ |
| **4. Delivery Tour** | **Nearest-Neighbor TSP** | Exact TSP is NP-hard; greedy nearest-neighbor produces a fast tour from the depot and back | $O(k^2)$ for $k$ stops |

---

## End-to-End Workflow

1. **Data Ingestion:** Reads `parcels.csv`, `nodes.csv`, and `roads.csv` into memory and builds an undirected adjacency list for the road graph.
2. **Urgency Sorting:** Sorts parcels by `(deadline_hr ascending, priority descending)` using Merge Sort.
3. **Knapsack Optimization:** Fills a `16 x 26` DP table (`dp[i][w]`) and backtracks from `dp[n][W]` to recover the optimal set of parcels within `25 kg`.
4. **All-Pairs Stop Distances:** Extracts unique destinations from the selected parcels and runs Dijkstra's algorithm from the depot and each stop.
5. **Route Construction:** Uses Nearest-Neighbor TSP on the Dijkstra distance matrix to visit every stop once and return to the depot.
6. **Verification & Benchmarking:** Runs automated correctness assertions and measures execution time across scaling input sizes (`n = 100` to `1,600`).

---

## Input Data Schema

- **`parcels.csv`** (`15` sample parcels: `P01`–`P15`)
  - `parcel_id`: Unique parcel identifier
  - `destination`: Target node in the road network
  - `weight_kg`: Parcel weight in kilograms
  - `priority`: Urgency/value score (`1` = low to `10` = high)
  - `deadline_hr`: Remaining hours until delivery deadline
- **`nodes.csv`** (`8` nodes: `Node 0` as Depot + `7` Gurugram delivery zones)
- **`roads.csv`** (`11` undirected road segments with distances in `km`)

---

## Repository Files

| File | Description |
| :--- | :--- |
| `courier_planner.py` | Complete Python implementation: CSV loader, Merge Sort, 0/1 Knapsack DP, Dijkstra's algorithm, Nearest-Neighbor TSP, unit tests, and benchmark suite |
| `parcels.csv` | Sample parcel dataset (`15` parcels) |
| `nodes.csv` | Sample road network vertices (`8` nodes) |
| `roads.csv` | Sample weighted road edges (`11` roads) |
| `output_trace.txt` | Full execution log containing the sorted parcel table, DP table, selected load, and final route |
| `runtime_chart.png` | `2 x 2` benchmark visualization comparing runtime growth across input sizes |

---

## Installation & Usage

### Prerequisites
- **Python 3.8+** (uses only standard library modules: `csv`, `heapq`, `random`, `time`, `sys`)
- **Matplotlib** (optional, required only for generating `runtime_chart.png`):
  ```bash
  pip install matplotlib