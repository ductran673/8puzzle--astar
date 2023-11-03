import sys
from collections import deque

class EightPuzzleProblem:
    def __init__(self, initial_state, goal_state):
        self.initial = initial_state
        self.goal = goal_state
        self.step_cost = 1

    def Action(self, state):
        possible_action = {'UP', 'DOWN', 'LEFT', 'RIGHT'}
        blank_id = state.index(0)
        if blank_id % 3 == 0:
            possible_action.remove('LEFT')
        if blank_id % 3 == 2:
            possible_action.remove('RIGHT')
        if blank_id / 3 < 1:
            possible_action.remove('UP')
        if blank_id / 3 >= 2:
            possible_action.remove('DOWN')
        return possible_action

    def Result(self, state, action):
        blank_id = state.index(0)
        state = list(state)  # Create a copy of the current state
        if action == 'UP':
            state[blank_id], state[blank_id - 3] = state[blank_id - 3], state[blank_id]
        if action == 'DOWN':
            state[blank_id], state[blank_id + 3] = state[blank_id + 3], state[blank_id]
        if action == 'LEFT':
           state[blank_id], state[blank_id - 1] = state[blank_id - 1], state[blank_id]
        if action == 'RIGHT':
            state[blank_id], state[blank_id + 1] = state[blank_id + 1],state[blank_id]
        return tuple(state)

    def Step_cost(self, cur_state, nx_state):
        return self.step_cost

    def Heuristic(self, state):
        # Thay đổi hàm này để tính toán giá trị heuristic, ví dụ: số lượng ô đã đặt sai vị trí so với mục tiêu.
        heuristic_value = 0
        for i in range(9):
            if state[i] != self.goal[i]:
                heuristic_value += 1
        return heuristic_value

    def Path_cost(self, cur_state, cur_cost, nx_state):
        return cur_cost + self.Step_cost(cur_state, nx_state)

    def f_value(self, state, g_value):
        return g_value + self.Heuristic(state)

    def Goal_test(self, state):
        return (state == self.goal)

class Node:
    def __init__(self, state, parent=None, action=None, cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost
        self.depth = parent.depth + 1 if parent else 0

    def Child_node(self, action, problem):
        return Node(problem.Result(self.state, action), self, action, self.cost + problem.Step_cost(self.state, problem.Result(self.state, action)))

    def Expand(self, problem):
        List_successor = []
        possible_action = problem.Action(self.state)
        for action in possible_action:
            List_successor.append(self.Child_node(action, problem))
        return List_successor

    def Solution(self):
        node, solution = self, []
        while node.parent:
            solution.append(node.action)
            node = node.parent
        return list(reversed(solution))

class EightPuzzleSolving:
    def __init__(self, problem):
        self.solution = self.a_star_search(problem).Solution()

    def a_star_search(self, problem):
        start_node = Node(problem.initial)
        frontier = [(problem.f_value(start_node.state, 0), start_node)]
        explored = set()

        while frontier:
            _, node = frontier.pop(0)  # Chọn nút có f(n) nhỏ nhất
            if problem.Goal_test(node.state):
                return node

            explored.add(node.state)
            for child in node.Expand(problem):
                if child.state not in explored:
                    g_value = node.cost + problem.Step_cost(node.state, child.state)
                    f_value = problem.f_value(child.state, g_value)
                    frontier.append((f_value, child))

        return None

if __name__ == '__main__':
    problem = EightPuzzleProblem(initial_state=(3, 1,0, 2, 6, 8,7,5,4), goal_state=(0, 1, 2, 3, 4, 5, 6, 7, 8))
    solving = EightPuzzleSolving(problem)
    print("Solution:", solving.solution)
   