"""
Mini-Courier Planner  (Capstone Assignment - Design & Analysis of Algorithms)

Pipeline:
  1. Load parcels, road graph
  2. Sort parcels by deadline / priority      -> Merge Sort
  3. Choose parcels for one trip              -> 0/1 Knapsack (Dynamic Programming)
  4. Shortest paths between stops             -> Dijkstra's Algorithm
  5. Delivery route for the whole trip        -> Nearest-Neighbor TSP approximation
  6. Runtime testing on small and large data  -> benchmark + runtime chart

Run:  python courier_planner.py            (full run: demo + tests + benchmark)
      python courier_planner.py --no-bench (skip the benchmark)
"""

import csv
import heapq
import itertools
import random
import sys
import time
from dataclasses import dataclass

DEPOT = 0
VAN_CAPACITY_KG = 25


# ----------------------------------------------------------------------
# Data design
# ----------------------------------------------------------------------
@dataclass
class Parcel:
    pid: str          # parcel id
    dest: int         # destination node in the road graph
    weight: int       # kg
    priority: int     # 1 (low) .. 10 (high) -> used as the "value" in knapsack
    deadline: int     # hours from now


def load_parcels(path):
    parcels = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            parcels.append(Parcel(r["parcel_id"], int(r["destination"]),
                                  int(r["weight_kg"]), int(r["priority"]),
                                  int(r["deadline_hr"])))
    return parcels


def load_names(path):
    with open(path, newline="") as f:
        return {int(r["node_id"]): r["name"] for r in csv.DictReader(f)}


def load_graph(path):
    """Undirected weighted graph as an adjacency list: {node: [(neighbour, km), ...]}"""
    graph = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            u, v, w = int(r["u"]), int(r["v"]), float(r["distance_km"])
            graph.setdefault(u, []).append((v, w))
            graph.setdefault(v, []).append((u, w))
    return graph


# ----------------------------------------------------------------------
# Phase 2a: Sorting  (Merge Sort - O(n log n), stable)
# ----------------------------------------------------------------------
def parcel_key(p):
    """Earlier deadline first; for equal deadlines, higher priority first."""
    return (p.deadline, -p.priority)


def merge_sort(items, key=lambda x: x):
    if len(items) <= 1:
        return items
    mid = len(items) // 2
    left = merge_sort(items[:mid], key)
    right = merge_sort(items[mid:], key)
    merged, i, j = [], 0, 0
    while i < len(left) and j < len(right):
        if key(left[i]) <= key(right[j]):      # <= keeps the sort stable
            merged.append(left[i]); i += 1
        else:
            merged.append(right[j]); j += 1
    merged.extend(left[i:])
    merged.extend(right[j:])
    return merged


def insertion_sort(items, key=lambda x: x):
    """O(n^2) baseline - used only to compare against merge sort."""
    a = list(items)
    for i in range(1, len(a)):
        cur, j = a[i], i - 1
        while j >= 0 and key(a[j]) > key(cur):
            a[j + 1] = a[j]
            j -= 1
        a[j + 1] = cur
    return a


# ----------------------------------------------------------------------
# Phase 2b: 0/1 Knapsack (Dynamic Programming) - O(n * W)
# ----------------------------------------------------------------------
def knapsack(parcels, capacity):
    """Pick the parcels with the highest total priority that fit in the van."""
    n = len(parcels)
    dp = [[0] * (capacity + 1) for _ in range(n + 1)]
    for i in range(1, n + 1):
        p = parcels[i - 1]
        for w in range(capacity + 1):
            dp[i][w] = dp[i - 1][w]                          # skip parcel i
            if p.weight <= w:                                # take parcel i
                dp[i][w] = max(dp[i][w], dp[i - 1][w - p.weight] + p.priority)

    # backtrack to find which parcels were chosen
    chosen, w = [], capacity
    for i in range(n, 0, -1):
        if dp[i][w] != dp[i - 1][w]:
            chosen.append(parcels[i - 1])
            w -= parcels[i - 1].weight
    chosen.reverse()
    return dp[n][capacity], chosen, dp


# ----------------------------------------------------------------------
# Phase 3a: Dijkstra's shortest path (min-heap) - O((V + E) log V)
# ----------------------------------------------------------------------
def dijkstra(graph, source):
    dist = {node: float("inf") for node in graph}
    prev = {node: None for node in graph}
    dist[source] = 0
    heap = [(0, source)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue                                         # stale entry
        for v, w in graph[u]:
            if d + w < dist[v]:
                dist[v] = d + w
                prev[v] = u
                heapq.heappush(heap, (dist[v], v))
    return dist, prev


def get_path(prev, target):
    path = []
    while target is not None:
        path.append(target)
        target = prev[target]
    return path[::-1]


# ----------------------------------------------------------------------
# Phase 3b: Nearest-Neighbor TSP approximation - O(k^2)
# ----------------------------------------------------------------------
def nearest_neighbor_route(stops, dist_matrix, start=DEPOT):
    """Always go to the closest unvisited stop, then return to the depot."""
    unvisited = set(stops) - {start}
    route, current, total = [start], start, 0
    while unvisited:
        nxt = min(unvisited, key=lambda s: dist_matrix[current][s])
        total += dist_matrix[current][nxt]
        route.append(nxt)
        unvisited.remove(nxt)
        current = nxt
    total += dist_matrix[current][start]
    route.append(start)
    return route, total


def plan_route(graph, stops):
    """Dijkstra from every stop -> distance matrix -> nearest-neighbor tour."""
    stops = sorted(set(stops) | {DEPOT})
    dist_matrix, prev_maps = {}, {}
    for s in stops:
        dist, prev = dijkstra(graph, s)
        dist_matrix[s] = dist
        prev_maps[s] = prev
    route, total = nearest_neighbor_route(stops, dist_matrix)
    return route, total, dist_matrix, prev_maps


# ----------------------------------------------------------------------
# Integrated pipeline
# ----------------------------------------------------------------------
def plan_courier(parcels, graph, capacity):
    sorted_parcels = merge_sort(parcels, parcel_key)
    best_value, chosen, dp = knapsack(sorted_parcels, capacity)
    stops = {p.dest for p in chosen}
    route, total, dist_matrix, prev_maps = plan_route(graph, stops)
    return sorted_parcels, best_value, chosen, dp, route, total, dist_matrix, prev_maps


def show_plan(parcels, graph, names, capacity):
    sorted_parcels, best_value, chosen, dp, route, total, dist_matrix, prev_maps = \
        plan_courier(parcels, graph, capacity)

    print("=" * 70)
    print("STEP 1: PARCELS SORTED BY DEADLINE (then priority)   [Merge Sort]")
    print("=" * 70)
    print(f"{'ID':<5}{'Dest':<18}{'Wt(kg)':<8}{'Priority':<10}{'Deadline(hr)'}")
    for p in sorted_parcels:
        print(f"{p.pid:<5}{names[p.dest]:<18}{p.weight:<8}{p.priority:<10}{p.deadline}")

    print("\n" + "=" * 70)
    print(f"STEP 2: PACKING PLAN  (van capacity = {capacity} kg)   [0/1 Knapsack DP]")
    print("=" * 70)
    if len(parcels) <= 20 and capacity <= 30:
        print("DP table (rows = parcels in sorted order, columns = capacity 0..%d):" % capacity)
        print("     " + "".join(f"{w:>3}" for w in range(capacity + 1)))
        for i, row in enumerate(dp):
            label = "--" if i == 0 else sorted_parcels[i - 1].pid
            print(f"{label:<5}" + "".join(f"{v:>3}" for v in row))
    print("\nSelected parcels:", ", ".join(p.pid for p in chosen))
    print("Total weight  :", sum(p.weight for p in chosen), "kg")
    print("Total priority:", best_value)
    left = [p.pid for p in sorted_parcels if p not in chosen]
    print("Left for next trip:", ", ".join(left))

    print("\n" + "=" * 70)
    print("STEP 3: SHORTEST PATHS FROM DEPOT   [Dijkstra]")
    print("=" * 70)
    for s in sorted({p.dest for p in chosen}):
        path = get_path(prev_maps[DEPOT], s)
        print(f"Depot -> {names[s]:<18} {dist_matrix[DEPOT][s]:>5.1f} km   "
              + " > ".join(names[n] for n in path))

    print("\n" + "=" * 70)
    print("STEP 4: FINAL DELIVERY ROUTE   [Nearest-Neighbor TSP]")
    print("=" * 70)
    print("Order of stops:", " -> ".join(names[n] for n in route))
    full_len = 0
    for a, b in zip(route, route[1:]):
        path = get_path(prev_maps[a], b)
        leg = dist_matrix[a][b]
        full_len += leg
        print(f"  {names[a]:<18} -> {names[b]:<18} {leg:>5.1f} km  (via "
              + " > ".join(names[n] for n in path) + ")")
    print(f"Total route length: {full_len:.1f} km")
    return route, total


# ----------------------------------------------------------------------
# Correctness checks
# ----------------------------------------------------------------------
def brute_force_knapsack(parcels, capacity):
    best = 0
    for r in range(len(parcels) + 1):
        for combo in itertools.combinations(parcels, r):
            if sum(p.weight for p in combo) <= capacity:
                best = max(best, sum(p.priority for p in combo))
    return best


def random_parcels(n, num_nodes, seed=1):
    rnd = random.Random(seed)
    return [Parcel(f"R{i}", rnd.randint(1, num_nodes - 1), rnd.randint(1, 20),
                   rnd.randint(1, 10), rnd.randint(1, 48)) for i in range(n)]


def random_graph(num_nodes, seed=1):
    """Connected random graph: a ring plus extra random roads."""
    rnd = random.Random(seed)
    graph = {i: [] for i in range(num_nodes)}

    def add(u, v):
        w = rnd.randint(1, 20)
        graph[u].append((v, w))
        graph[v].append((u, w))

    for i in range(num_nodes):
        add(i, (i + 1) % num_nodes)
    for _ in range(num_nodes * 2):
        u, v = rnd.sample(range(num_nodes), 2)
        add(u, v)
    return graph


def run_tests(graph):
    print("=" * 70)
    print("CORRECTNESS CHECKS")
    print("=" * 70)
    # 1. Merge sort vs Python's sorted()
    data = random_parcels(200, 10)
    assert merge_sort(data, parcel_key) == sorted(data, key=parcel_key)
    assert insertion_sort(data, parcel_key) == sorted(data, key=parcel_key)
    print("PASS  merge sort and insertion sort match Python sorted() on 200 parcels")

    # 2. Knapsack vs brute force
    for seed in range(5):
        small = random_parcels(12, 5, seed)
        best, _, _ = knapsack(small, 30)
        assert best == brute_force_knapsack(small, 30)
    print("PASS  knapsack DP equals brute force on 5 random sets of 12 parcels")

    # 3. Dijkstra on the sample graph (hand-checked: Depot->Sector 14 = 4 km,
    #    Depot->MG Road = 7 km via Sector 14, Depot->Sector 56 = 14 km)
    dist, _ = dijkstra(graph, DEPOT)
    assert dist[1] == 4 and dist[4] == 7 and dist[7] == 14
    print("PASS  Dijkstra gives the hand-calculated distances on the sample graph")

    # 4. Route visits every stop exactly once and returns to the depot
    route, _, _, _ = plan_route(graph, {1, 3, 5, 7})[0:4]
    assert route[0] == route[-1] == DEPOT and sorted(route[:-1]) == [0, 1, 3, 5, 7]
    print("PASS  route starts/ends at the depot and visits each stop exactly once")

    # 5. How close is nearest-neighbor to the best possible tour? (brute force, 5 stops)
    stops = [1, 2, 3, 4, 6]
    _, nn_len, dm, _ = plan_route(graph, set(stops))
    best_len = min(sum(dm[a][b] for a, b in zip((DEPOT,) + p, p + (DEPOT,)))
                   for p in itertools.permutations(stops))
    print(f"INFO  nearest-neighbor tour = {nn_len:.0f} km, best possible tour = {best_len:.0f} km "
          f"({(nn_len - best_len) / best_len * 100:.1f}% longer)")


# ----------------------------------------------------------------------
# Phase 4: Performance testing
# ----------------------------------------------------------------------
def best_time(fn, repeat=3):
    """Run fn a few times and return the fastest time in seconds."""
    times = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        fn()
        times.append(time.perf_counter() - t0)
    return min(times)


def benchmark():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sizes = [100, 200, 400, 800, 1600]
    route_sizes = [20, 40, 80, 160, 320]
    res = {"merge": [], "insertion": [], "knap": [], "dijk": [], "route": []}

    print("\n" + "=" * 70)
    print("RUNTIME TESTING  (best of 3 runs, seconds)")
    print("=" * 70)
    print(f"{'n':>6}{'Merge sort':>14}{'Insertion sort':>16}{'Knapsack DP':>14}{'Dijkstra':>12}")
    for n in sizes:
        parcels = random_parcels(n, n)
        g = random_graph(n)
        res["merge"].append(best_time(lambda: merge_sort(parcels, parcel_key)))
        res["insertion"].append(best_time(lambda: insertion_sort(parcels, parcel_key)))
        res["knap"].append(best_time(lambda: knapsack(parcels, 500), repeat=1))
        res["dijk"].append(best_time(lambda: dijkstra(g, 0)))
        print(f"{n:>6}{res['merge'][-1]:>14.5f}{res['insertion'][-1]:>16.5f}"
              f"{res['knap'][-1]:>14.5f}{res['dijk'][-1]:>12.5f}")

    print(f"\n{'stops':>6}{'Dijkstra x k + NN-TSP (s)':>28}")
    for k in route_sizes:
        g = random_graph(k)
        res["route"].append(best_time(lambda: plan_route(g, set(range(k))), repeat=1))
        print(f"{k:>6}{res['route'][-1]:>28.5f}")

    # runtime chart (2 x 2)
    fig, ax = plt.subplots(2, 2, figsize=(11, 8))
    ax[0, 0].plot(sizes, res["merge"], "o-", label="Merge sort  O(n log n)")
    ax[0, 0].plot(sizes, res["insertion"], "s--", label="Insertion sort  O(n^2)")
    ax[0, 0].set_title("Sorting parcels"); ax[0, 0].set_xlabel("number of parcels (n)")
    ax[0, 0].legend()
    ax[0, 1].plot(sizes, res["knap"], "o-", color="tab:green")
    ax[0, 1].set_title("0/1 Knapsack DP (capacity 500)  O(nW)")
    ax[0, 1].set_xlabel("number of parcels (n)")
    ax[1, 0].plot(sizes, res["dijk"], "o-", color="tab:red")
    ax[1, 0].set_title("Dijkstra (one source)  O((V+E) log V)")
    ax[1, 0].set_xlabel("number of nodes (V)")
    ax[1, 1].plot(route_sizes, res["route"], "o-", color="tab:purple")
    ax[1, 1].set_title("Route planning: Dijkstra from every stop + NN-TSP")
    ax[1, 1].set_xlabel("number of stops (k)")
    for a in ax.flat:
        a.set_ylabel("time (seconds)"); a.grid(alpha=0.3)
    fig.suptitle("Mini-Courier Planner - runtime of major steps", fontsize=13)
    fig.tight_layout()
    fig.savefig("runtime_chart.png", dpi=150)
    print("\nRuntime chart saved as runtime_chart.png")


# ----------------------------------------------------------------------
if __name__ == "__main__":
    names = load_names("nodes.csv")
    graph = load_graph("roads.csv")
    parcels = load_parcels("parcels.csv")

    show_plan(parcels, graph, names, VAN_CAPACITY_KG)
    print()
    run_tests(graph)
    if "--no-bench" not in sys.argv:
        benchmark()
