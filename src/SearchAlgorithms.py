from math import sqrt
from Space import *
from Constants import *


def DFS(g: Graph, sc: pygame.Surface):
    print('Implement DFS algorithm')

    open_set = [g.start.value]
    closed_set = []
    father = [-1]*g.get_len()

    # TODO: Implement DFS algorithm using open_set, closed_set, and father
    path = []

    while len(open_set) > 0:
        node_id = open_set.pop()
        node = g.get_node(node_id)

        if node_id in closed_set:
            continue

        # If this node is the goal node, return the path.
        if g.is_goal(node):
            while node_id != -1:
                path.append(node_id)
                node_id = father[node_id]
            break

        closed_set.append(node_id)
        node.set_color(yellow)
        g.draw(sc)
        neighbors = g.get_neighbors(node)
        for neighbor in neighbors:
            if (not neighbor.value in closed_set) and (not neighbor.value in open_set):
                open_set.append(neighbor.value)
                father[neighbor.value] = node_id
                neighbor.set_color(red)

        node.set_color(blue)

    print_path(g, sc, path)


def BFS(g: Graph, sc: pygame.Surface):
    print('Implement BFS algorithm')

    open_set = [g.start.value]
    closed_set = []
    father = [-1]*g.get_len()

    # TODO: Implement BFS algorithm using open_set, closed_set, and father
    path = []

    while len(open_set) > 0:
        node_id = open_set.pop(0)
        node = g.get_node(node_id)

        if node_id in closed_set:
            continue

        # If this node is the goal node, return the path.
        if g.is_goal(node):
            while node_id != -1:
                path.append(node_id)
                node_id = father[node_id]
            break

        closed_set.append(node_id)
        node.set_color(yellow)
        g.draw(sc)
        neighbors = g.get_neighbors(node)
        for neighbor in neighbors:
            if (not neighbor.value in closed_set) and (not neighbor.value in open_set):
                open_set.append(neighbor.value)
                father[neighbor.value] = node_id
                neighbor.set_color(red)

        node.set_color(blue)

    print_path(g, sc, path)


def UCS(g: Graph, sc: pygame.Surface):
    print('Implement UCS algorithm')

    open_set = {}
    open_set[g.start.value] = 0
    closed_set: list[int] = []
    father = [-1]*g.get_len()
    cost = [100_000]*g.get_len()
    cost[g.start.value] = 0

    # TODO: Implement UCS algorithm using open_set, closed_set, and father
    # assume that the cost between every 2 vertices is equal to 1
    path = []

    while len(open_set) > 0:
        # dequeue the open set to get lowest cost node
        asc_open_set = sorted(open_set.items(), key=lambda x: x[1])
        node_id = asc_open_set[0][0]
        del open_set[node_id]
        node = g.get_node(node_id)

        if node_id in closed_set:
            continue

        # If this node is the goal node, return the path.
        if g.is_goal(node):
            while node_id != -1:
                path.append(node_id)
                node_id = father[node_id]
            break

        closed_set.append(node_id)

        node.set_color(yellow)
        g.draw(sc)

        neighbors = g.get_neighbors(node)
        for neighbor in neighbors:

            if neighbor.value in closed_set:
                continue

            cost[neighbor.value] = cost[node_id] + 1
            neighbor.set_color(red)

            #
            if (not neighbor.value in open_set):
                father[neighbor.value] = node_id
                open_set[neighbor.value] = cost[neighbor.value]

        node.set_color(blue)

    print_path(g, sc, path)


def euclidean_distance(first_node: Node, second_node: Node) -> float:
    dx = first_node.x - second_node.x
    dy = first_node.y - second_node.y

    return sqrt(dx * dx + dy * dy)


def diagonal_distance(first_node: Node, second_node: Node) -> float:
    dx = abs(first_node.x - second_node.x)
    dy = abs(first_node.y - second_node.y)
    D = 1.0
    D2 = sqrt(2.0)

    return D * (dx + dy) + (D2 - 2*D) * min(dx, dy)


def octile_distance(first_node: Node, second_node: Node) -> float:
    dx = abs(first_node.x - second_node.x)
    dy = abs(first_node.y - second_node.y)

    return max(dx, dy) + (sqrt(2.0) - 1) * min(dx, dy)


def GBFS(g: Graph, sc: pygame.Surface):
    print('Implement GBFS algorithm')

    open_set = [g.start.value]
    closed_set = []
    father = [-1]*g.get_len()

    # TODO: Implement BFS algorithm using open_set, closed_set, and father
    path = []

    def heuristic(q: Node, func=euclidean_distance):
        return func(q, g.goal)

    while len(open_set) > 0:
        min_h = 100_000_000
        for q in open_set:
            h = heuristic(g.get_node(q))
            if h < min_h:
                node_id = q
                min_h = h

        node = g.get_node(node_id)
        open_set.remove(node_id)

        if node_id in closed_set:
            continue

        # If this node is the goal node, return the path.
        if g.is_goal(node):
            while node_id != -1:
                path.append(node_id)
                node_id = father[node_id]
            break

        closed_set.append(node_id)
        node.set_color(yellow)
        g.draw(sc)
        neighbors = g.get_neighbors(node)
        for neighbor in neighbors:
            if (not neighbor.value in closed_set) and (not neighbor.value in open_set):
                open_set.append(neighbor.value)
                father[neighbor.value] = node_id
                neighbor.set_color(red)

        node.set_color(blue)

    print_path(g, sc, path)


def AStar(g: Graph, sc: pygame.Surface):
    print('Implement A* algorithm')

    open_set = {}
    open_set[g.start.value] = 0
    closed_set: list[int] = []
    father = [-1]*g.get_len()
    cost = [100_000]*g.get_len()
    cost[g.start.value] = 0

    # TODO: Implement A* algorithm using open_set, closed_set, and father
    path = []

    # Mention the heuristics function as "func" parameter for this functions
    def heuristic(q: Node, func=euclidean_distance):
        return func(q, g.goal)

    while len(open_set) > 0:
        # get the f = h + g for every node in open_set
        asc_open_set = sorted(
            open_set.items(), key=lambda x: x[1] + heuristic(g.get_node(x[0])))

        # dequeue the open set to get lowest f node
        node_id = asc_open_set[0][0]
        node = g.get_node(node_id)
        del open_set[node_id]

        if node_id in closed_set:
            continue

        # If this node is the goal node, return the path.
        if g.is_goal(node):
            while node_id != -1:
                path.append(node_id)
                node_id = father[node_id]
            break

        closed_set.append(node_id)

        node.set_color(yellow)
        g.draw(sc)

        neighbors = g.get_neighbors(node)
        for neighbor in neighbors:

            if neighbor.value in closed_set:
                continue

            cost[neighbor.value] = cost[node_id] + 1
            neighbor.set_color(red)

            if (not neighbor.value in open_set) or (open_set[neighbor.value] > cost[neighbor.value]):
                father[neighbor.value] = node_id
                open_set[neighbor.value] = cost[neighbor.value]

        node.set_color(blue)

    print_path(g, sc, path)


# print the back tracking path
def print_path(g: Graph, sc: pygame.Surface, path: list):
    total_cost = 0
    g.start.set_color(orange)
    g.goal.set_color(purple)
    for i in range(1, len(path)):
        total_cost += 1
        cur = g.get_node(path[i])
        cur.set_color(grey)
        father_node = g.get_node(path[i - 1])
        g.start.set_color(orange)
        g.goal.set_color(purple)
        pygame.draw.line(sc, green, [cur.x, cur.y], [
                         father_node.x, father_node.y], 5)

        g.draw(sc)

    print(f'Path: {path}')
    print(f'Total cost: {total_cost}')
