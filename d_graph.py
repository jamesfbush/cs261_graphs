# Course: CS261 - Data Structures
# Author:       James Bush
# Assignment:   6
# Description:  Implementation of directed graph supporting folllowing methods:
#               add_vertex(), add_edge(), remove_edge(), get_vertices(),
#               get_edges(), is_valid_path(), dfs(), ​ bfs(), has_cycle(),
#               dijkstra().


class DirectedGraph:
    """
    Class to implement directed weighted graph
    - duplicate edges not allowed
    - loops not allowed
    - only positive edge weights
    - vertex names are integers
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency matrix
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.v_count = 0
        self.adj_matrix = []

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            v_count = 0
            for u, v, _ in start_edges:
                v_count = max(v_count, u, v)
            for _ in range(v_count + 1):
                self.add_vertex()
            for u, v, weight in start_edges:
                self.add_edge(u, v, weight)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        if self.v_count == 0:
            return 'EMPTY GRAPH\n'
        out = '   |'
        out += ' '.join(['{:2}'.format(i) for i in range(self.v_count)]) + '\n'
        out += '-' * (self.v_count * 3 + 3) + '\n'
        for i in range(self.v_count):
            row = self.adj_matrix[i]
            out += '{:2} |'.format(i)
            out += ' '.join(['{:2}'.format(w) for w in row]) + '\n'
        out = f"GRAPH ({self.v_count} vertices):\n{out}"
        return out

    # ------------------------------------------------------------------ #

    def add_vertex(self) -> int:
        """
        Add a new vertex to the graph.
        Return the count of vertices in the graph following addition of new
        vertex.
        """

        # Add new column to each vertex
        for vertex in self.adj_matrix:
            vertex.append(0)

        # Increment count of vertices
        self.v_count += 1

        # Add new row to graph for new vertex
        self.adj_matrix.append([0 for vertex in range(self.v_count)])

        # Return vertex count
        return self.v_count


    def add_edge(self, src: int, dst: int, weight=1) -> None:
        """
        Take src, dst, and weight.
        Add new edge to the graph connecting src and dst.
        If either (or both) src and dst do not exist in graph,
        or if​ weight​ is not a positive integer, or if src​ and ​dst​ refer
        to the same vertex, method does nothing. If an edge already exists,
        the method will update its weight.
        """

        # Source out of range
        if src > (self.v_count - 1) or src < 0:
            return

        # Destination out of range
        if dst > (self.v_count -1) or dst < 0:
            return

        # If source and destination are same
        if src == dst:
            return

        # Weight not positive
        if weight < 0:
            return

        # Otherwise, return weight
        self.adj_matrix[src][dst] = weight

    def remove_edge(self, src: int, dst: int) -> None:
        """
        Take src and dst. Remove edge between src and dst.
        If either (or both) vertices do not exist in the graph,
        or if there is no edge between them, the method does nothing
        """

        # Source out of range
        if src > (self.v_count - 1) or src < 0:
            return

        # Destination out of range
        if dst > (self.v_count -1) or dst < 0:
            return

        # If source and destination are same
        if src == dst:
            return

        # Otherwise, remove edge
        self.adj_matrix[src][dst] = 0

    def get_vertices(self) -> []:
        """
        Return a list of vertices of the graph.
        """

        return [vertex for vertex in range(self.v_count)]

    def get_edges(self, no_weights=False) -> []:
        """
        Return list of edges in the graph. Each edge is: (src, dst, weight).
        Optionally take a parameter (if set True) to not return weights.
        """

        # Declare edge list
        edges = []

        # Obtain weight (> 0) from src, dst in matrix
        for src in range(self.v_count):
            for dst in range(len(self.adj_matrix[src])):
                if self.adj_matrix[src][dst] > 0:

                    # If no_weights set, True, return only edge
                    if no_weights is True:
                        edges.append((src, dst))

                    # Else, return with weight as well
                    else:
                        edges.append((src, dst, self.adj_matrix[src][dst]))

        # Edge list
        return edges

    def is_valid_path(self, path: []) -> bool:
        """
        Take list vertex indices and return True if sequence of vertices is a
        valid path in the graph. Empty path is considered valid.
        """

        # Declare edges and queue
        edges = self.get_edges(True)
        queue = [i for i in path[::-1]]

        # If queue empty, path valid
        if not queue:
            return True

        # If only one source in path, path valid
        if len(queue) == 1 and sum(self.adj_matrix[path[0]]) > 1:
            return True

        # Paths with at least one src, dst
        src, dst = queue.pop(), queue.pop()

        # Preliminiary check if first edge valid
        if (src, dst) not in edges:
            return False

        # Beyond first edge
        while queue:
            if (src, dst) in edges:
                src = dst
                dst = queue.pop()
            if (src, dst) not in edges:
                return False

        # Valid path
        return True


    def dfs(self, v_start, v_end=None) -> []:
        """
        Take v_start (required) and v_end (optional). Beginning at v_start,
        perform depth-first search (DFS) in the graph and return list of
        vertices visited during DFS, in the order visited.
        """

        # If the starting vertex is not in the graph, return empty list
        if v_start > (len(self.adj_matrix) - 1) or v_start < 0:
            return []

        # 1. Initialize an empty set of visited cycle vertices.
        visited = []

        # 2. Initialize an empty stack (DFS). Add v_start to the stack.
        stack = [v_start]

        # 3. If the stack/queue is not empty, pop/dequeue a vertex v.
        while stack:
            v = stack.pop()

            # 4. Perform any desired processing on v.
            # ...e.g., check if v meets a desired condition.
            if v_end is not None:
                if v_end >= 0 and v_end <= (len(self.adj_matrix) - 1):
                    if v == v_end:
                        visited.append(v)
                        return visited

            # 5. (DFS only): If v is not in the set of visited vertices:
            if v not in visited:

                # 1. Add v to the set of visited vertices.
                visited.append(v)

                # 2. Push each vertex that is direct successor of v to the stack.
                level = []
                for adjacent in range(len(self.adj_matrix[v])):
                    if self.adj_matrix[v][adjacent] > 0:
                        level.append(adjacent)
                level.sort(reverse=True)
                for i in level:
                    stack.append(i)

        # Return visited vertices in DFS when stack empty
        return visited


    def bfs(self, v_start, v_end=None) -> []:
        """
        Take v_start (required) and v_end (optional). Beginning at v_start,
        perform breadth-first search (BFS) in the graph and return list of
        vertices visited during BFS, in the order visited.
        """

        # If the starting vertex is not in the graph, return empty list
        if v_start > (len(self.adj_matrix) - 1) or v_start < 0:
            return []

        # 1. Initialize an empty set of visited cycle vertices.
        visited = []

        # 2. Initialize an empty queue (BFS). Add vi to the stack/queue.
        queue = [v_start]

        # 3. If the stack/queue is not empty, pop/dequeue a vertex v.
        while queue:
            v = queue[0]
            del queue[0]            #

            # 4. Perform any desired processing on v.
            if v_end is not None:
                if v_end >= 0 and v_end <= (len(self.adj_matrix) - 1):
                    if v == v_end:
                        visited.append(v)
                        return visited

            # Add v to the set of visited vertices.
            if v not in visited:
                visited.append(v)

            # For each direct successor of v:
            level = []
            for adjacent in range(len(self.adj_matrix[v])):
                if self.adj_matrix[v][adjacent] > 0:
                    level.append(adjacent)
            level.sort()
            for i in level:
                if i not in visited:
                    queue.append(i)

        #  Return visited vertices in BFS when queue empty
        return visited

    def has_cycle(self):
        """
        Return True if there is at least one cycle in the graph.
        If the graph is acyclic,return False
        """

        # Examine each vertex
        for vertex in self.get_vertices():

            # ...and the vertices adjacent to it
            adjacent = self.get_adjacent(vertex)

            # ...if there is a path from the adjacent back to origin
            for i in adjacent:
                path = self.dfs(i)

                # ...then we have a cycle
                if vertex in path:
                    return True

        # or not.
        return False


    def dijkstra(self, src: int) -> []:
        """
        Take src. Employ Dijkstra's algorithm to compute distance/cost of
        shortest path from a given vertex to all other vertices in the graph.
        Return list with distance/cost per each vertex in the graph.
        If a certain vertex is not reachable from src, returned value is
        float('inf').
        """

        # Import heapq and determine reachable vertices from src
        import heapq
        reachable = self.dfs(src)

        #1. Initialize an empty map/hash table representing visited vertices
        #  Key is the vertex/v, value is the min distance d to vertex v.
        visited = {}

        #2. Initialize empty priority queue; insert src w/ distance/priority 0.
        priority_queue = [(0, src)]

        #3. While the priority queue is not empty:
        while priority_queue:

            # Remove first vertex (v), and distance/priority (d).
            d, v = heapq.heappop(priority_queue)

            # If v is not in the dict of visited vertices:
            if v not in visited.keys():

                # Add v to visited dict with distance d.
                visited[v] = d

                # For each direct successor v_i of v:
                for v_i in self.get_adjacent(v):

                    # Let d_i equal the cost associated with edge (v, v_i).
                    d_i = self.adj_matrix[v][v_i] #maybe grab from edges?

                    # Insert v_i in priority queue w/ distance/priority d + d_i.
                    heapq.heappush(priority_queue, ((d + d_i), v_i))

        # If vertex is not reachable, set its distance to infinity
        for vertex in self.get_vertices():
            if vertex not in reachable:
                visited[vertex] = float('inf')

        # Sort the dictionary by vertex for return
        visited = sorted([(v, d) for (v, d) in visited.items()])

        # Return the distances
        return [dist[1] for dist in visited]

    def get_adjacent(self, vertex):
        """
        Helper. Take vertex, return adjacent (non-null-weight) vertices by index
        in a list.
        """

        return [adjacent for adjacent \
            in range(len(self.adj_matrix[vertex])) \
            if self.adj_matrix[vertex][adjacent] > 0]

# TESTING
if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = DirectedGraph()
    print(g)
    for _ in range(5):
        g.add_vertex()
    print(g)

    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    for src, dst, weight in edges:
        g.add_edge(src, dst, weight)
    print(g)


    print("\nPDF - method get_edges() example 1")
    print("----------------------------------")
    g = DirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    test_cases = [[0, 1, 4, 3], [1, 3, 2, 1], [0, 4], [4, 0], [], [2]]
    for path in test_cases:
        print(path, g.is_valid_path(path))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for start in range(5):
        print(f'{start} DFS:{g.dfs(start)} BFS:{g.bfs(start)}')


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)

    edges_to_remove = [(3, 1), (4, 0), (3, 2)]
    for src, dst in edges_to_remove:
        g.remove_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')

    edges_to_add = [(4, 3), (2, 3), (1, 3), (4, 0)]
    for src, dst in edges_to_add:
        g.add_edge(src, dst)
        print(g.get_edges(), g.has_cycle(), sep='\n')
    print('\n', g)


    print("\nPDF - dijkstra() example 1")
    print("--------------------------")
    edges = [(0, 1, 10), (4, 0, 12), (1, 4, 15), (4, 3, 3),
             (3, 1, 5), (2, 1, 23), (3, 2, 7)]
    g = DirectedGraph(edges)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
    g.remove_edge(4, 3)
    print('\n', g)
    for i in range(5):
        print(f'DIJKSTRA {i} {g.dijkstra(i)}')
