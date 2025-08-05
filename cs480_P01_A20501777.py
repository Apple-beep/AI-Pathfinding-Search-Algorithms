import csv
import sys
import time
from queue import PriorityQueue

class SearchNode:
    """
    Represents a node in the search tree.

    Attributes:
      current: The current state.
      parent_node: The parent SearchNode.
      step: A tuple (next_state, cost) representing the action.
      total_cost: The cumulative cost from the start node.
    """
    def __init__(self, current, parent_node=None, step=None, total_cost=0):
        self.current = current
        self.parent_node = parent_node
        self.step = step
        self.total_cost = total_cost

    def __lt__(self, other):
        return self.total_cost < other.total_cost

class SearchProblem:
    """
    Defines the search problem.

    Attributes:
      start_state: The initial state.
      target_state: The goal state.
      drive_map: A dictionary representing driving distances between states.
      heuristic_map: A dictionary representing straight-line distances (heuristic values).
    """
    def __init__(self, start_state, target_state, drive_map, heuristic_map):
        self.start_state = start_state
        self.target_state = target_state
        self.drive_map = drive_map
        self.heuristic_map = heuristic_map

    def reached_goal(self, state):
        return state == self.target_state

    def possible_steps(self, state):
        return self.drive_map[state]

    def next_state(self, state, step):
        return step[0]

    def step_cost(self, state, step, new_state):
        return step[1]

def search_best_first(problem: SearchProblem, eval_fn):
    """General best-first search using the given evaluation function."""
    root = SearchNode(problem.start_state)
    open_list = PriorityQueue()
    open_list.put((eval_fn(problem, root), root))
    seen = {problem.start_state: root}
    start_clock = time.time()
    nodes_expanded = 0

    while not open_list.empty():
        _, current_node = open_list.get()
        if problem.reached_goal(current_node.current):
            end_clock = time.time()
            return current_node, nodes_expanded, end_clock - start_clock
        nodes_expanded += 1

        for successor in expand_node(problem, current_node):
            succ_state = successor.current
            if succ_state not in seen or successor.total_cost < seen[succ_state].total_cost:
                seen[succ_state] = successor
                open_list.put((eval_fn(problem, successor), successor))

    end_clock = time.time()
    return None, 0, end_clock - start_clock

def expand_node(problem: SearchProblem, node: SearchNode):
    """Generate successor nodes from the current node."""
    current_state = node.current
    for action in problem.possible_steps(current_state):
        next_st = problem.next_state(current_state, action)
        accumulated_cost = node.total_cost + problem.step_cost(current_state, action, next_st)
        yield SearchNode(current=next_st, parent_node=node, step=action, total_cost=accumulated_cost)

def load_drive_map(file_name):
    """Load the driving distance map from a CSV file."""
    with open(file_name, newline="", encoding="utf-8") as csvfile:
        csv_reader = csv.reader(csvfile)
        headers = next(csv_reader)[1:]
        drive_map = {state: [] for state in headers}
        for row in csv_reader:
            state_label = row[0]
            for header, cost_value in zip(headers, row[1:]):
                if cost_value != "-1" and state_label != header:
                    drive_map[state_label].append((header, int(cost_value)))
    return drive_map

def load_heuristic_map(file_name):
    """Load the heuristic (straight-line) distance map from a CSV file."""
    with open(file_name, newline="", encoding="utf-8") as csvfile:
        csv_reader = csv.reader(csvfile)
        headers = next(csv_reader)[1:]
        heuristic_map = {state: {} for state in headers}
        for row in csv_reader:
            state_label = row[0]
            for header, cost_value in zip(headers, row[1:]):
                heuristic_map[state_label][header] = int(cost_value)
    return heuristic_map

def greedy_function(problem: SearchProblem, node: SearchNode):
    """
    Evaluation function for Greedy Best-First Search.
    Returns the heuristic distance from the current node to the target.
    """
    return problem.heuristic_map[node.current][problem.target_state]

def astar_function(problem: SearchProblem, node: SearchNode):
    """
    Evaluation function for A* Search.
    Returns the sum of the cost so far and the heuristic distance to the target.
    """
    return node.total_cost + problem.heuristic_map[node.current][problem.target_state]

def build_path(solution_node):
    """Reconstruct the path from the solution node back to the start."""
    path_list = []
    while solution_node:
        path_list.append(solution_node.current)
        solution_node = solution_node.parent_node
    path_list.reverse()
    return path_list

def validate_input(arguments):
    if len(arguments) != 3:
        print("ERROR: Not enough or too many input arguments.")
        return False
    return True

if __name__ == "__main__":
    arguments = sys.argv
    if not validate_input(arguments):
        sys.exit(1)

    try:
        # Load maps from CSV files (files must be in the same directory)
        drive_map = load_drive_map("driving_.csv")
        heuristic_map = load_heuristic_map("straightline.csv")

        init_state = arguments[1].upper()
        targ_state = arguments[2].upper()

        prob_instance = SearchProblem(init_state, targ_state, drive_map, heuristic_map)

        print("Pathan, Musharaf khan, A20501777 solution:")
        print(f"Initial state: {init_state}")
        print(f"Goal state: {targ_state}\n")

        # Run Greedy Best-First Search
        greedy_solution, greedy_expansions, greedy_duration = search_best_first(prob_instance, greedy_function)
        # Run A* Search
        astar_solution, astar_expansions, astar_duration = search_best_first(prob_instance, astar_function)

        if greedy_solution:
            greedy_path = build_path(greedy_solution)
            print("Greedy Best First Search:")
            print(f"Solution: {greedy_path}")
            print(f"Number of expanded nodes: {greedy_expansions}")
            print(f"Number of stops on a path: {len(greedy_path)}")
            print(f"Execution time: {greedy_duration:.6f} seconds")
            print(f"Complete path cost: {greedy_solution.total_cost}\n")
        else:
            print("Greedy Best First Search:")
            print("Solution: NO SOLUTION FOUND")
            print("Number of stops on a path: 0")
            print(f"Execution time: {greedy_duration:.6f} seconds")
            print("Complete path cost: 0\n")

        if astar_solution:
            astar_path = build_path(astar_solution)
            print("A* Search:")
            print(f"Solution: {astar_path}")
            print(f"Number of expanded nodes: {astar_expansions}")
            print(f"Number of stops on a path: {len(astar_path)}")
            print(f"Execution time: {astar_duration:.6f} seconds")
            print(f"Complete path cost: {astar_solution.total_cost}")
        else:
            print("A* Search:")
            print("Solution: NO SOLUTION FOUND")
            print("Number of stops on a path: 0")
            print(f"Execution time: {astar_duration:.6f} seconds")
            print("Complete path cost: 0")
    except Exception:
        print("Solution: NO SOLUTION FOUND")
        print("Number of stops on a path: 0")
        print("Execution time: 0 seconds")
        print("Complete path cost: 0")
