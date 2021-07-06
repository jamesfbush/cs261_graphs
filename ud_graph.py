# Course: CS261 - Data Structures
# Author:       James Bush
# Assignment:   5
# Description:  Implementation of undirected graph supporting following methods:
#               add_vertex(), add_edge(), remove_edge(), remove_vertex(),
#               get_vertices(), get_edges(), is_valid_path(), dfs(), bfs(),
#               count_connected_components(), has_cycle().


class UndirectedGraph:
    """
    Class to implement undirected graph
    - duplicate edges not allowed
    - loops not allowed
    - no edge weights
    - vertex names are strings
    """

    def __init__(self, start_edges=None):
        """
        Store graph info as adjacency list
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        self.adj_list = dict()

        # populate graph with initial vertices and edges (if provided)
        # before using, implement add_vertex() and add_edge() methods
        if start_edges is not None:
            for u, v in start_edges:
                self.add_edge(u, v)

    def __str__(self):
        """
        Return content of the graph in human-readable form
        DO NOT CHANGE THIS METHOD IN ANY WAY
        """
        out = [f'{v}: {self.adj_list[v]}' for v in self.adj_list]
        out = '\n  '.join(out)
        if len(out) < 70:
            out = out.replace('\n  ', ', ')
            return f'GRAPH: {{{out}}}'
        return f'GRAPH: {{\n  {out}}}'

    # ------------------------------------------------------------------ #


    def add_vertex(self, v: str) -> None:
        """
        Take vertex (v) and add to the graph. Vertex names can be any
        string. If vertex with the same name is already present in the graph,
        the method does nothing.
        """
        #if vertex in graph, do nothing
        if v in self.adj_list.keys():
            return

        # otherwise, add vertex {name:}
        self.adj_list[v] = []


    def add_edge(self, u: str, v: str) -> None:
        """
        Take two vertices (u, v) and a new edge to the graph, connecting two
        vertices. If either (or both) vertex names do not exist in the
        graph, this method will first create them and then create an edge
        between them.
        """

        # If u == v, do nothing
        if u == v:
            return

        # If one or more vertices not in graph
        [self.add_vertex(vertex) for vertex in (u, v) if vertex not in \
            self.adj_list.keys()]

        # Add edge if not already in edge list
        if v not in self.adj_list[u] and v != u and v in self.adj_list.keys():
            self.adj_list[u].append(v)

        if u not in self.adj_list[v] and v != u and u in self.adj_list.keys():
            self.adj_list[v].append(u)


    def remove_edge(self, v: str, u: str) -> None:
        """
        Take two vertices (u, v) and remove the edge between them. If either (or
        both) vertices do not exist in the graph, or if there is no edge
        between them, the method does nothing.
        """

        # If vertices are same
        if u == v:
            return

        # If either or both vertices don't exist in the graph, do nothing
        if v not in self.adj_list.keys() or u not in self.adj_list.keys():
            return

        # If there is no edge between the vertices, do nothing
        if v not in self.adj_list[u]:
            return

        # Else, remove the edge
        self.adj_list[u].remove(v)
        self.adj_list[v].remove(u)


    def remove_vertex(self, v: str) -> None:
        """
        Take vertex name (v), remove from graph, as well as all edges.
        If the vertex does not exist, the method does nothing.
        """

        # If vertex in graph
        if v in self.adj_list.keys():

            #Remove the vertex's edge w/ all corresponding vertices
            [self.remove_edge(v, i) for i in list(self.adj_list[v])]

            #Delete vertex
            del self.adj_list[v]


    def get_vertices(self) -> []:
        """
        Return an unordered list of vertices of the graph.
        """

        return list(self.adj_list.keys())


    def get_edges(self) -> []:
        """
        Return unordered list of edges in the graph.
        """

        # Declare edge list
        edge_list = []

        # Evaluate each vertex in graph
        for key in self.adj_list.keys():

            # Evaluate each corresponding edge vertex in vertex
            for value in self.adj_list[key]:

                # Declare edge of vertex/corresponding vertex
                edge = tuple(sorted([key, value]))

                # Add edge to list if not existing and return
                if edge not in edge_list:
                    edge_list.append(edge)

        # Return unorder list of edges
        return edge_list


    def is_valid_path(self, path: []) -> bool:
        """
        Take path. Return true if provided path is valid within the graph,
        False otherwise
        """

        # Empty path is valid
        if len(path) == 0:
            return True

        # Single-node path
        if len(path) == 1 and path[0] not in self.adj_list:
            return False

        # Paths larger than single node
        index = 0
        while index <= len(path) - 2:
            if path[index+1] in self.adj_list[path[index]]:
                index += 1
            else:
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
        if v_start not in self.adj_list:
            return []

        #1. Initialize an empty set of visited vertices.
        visited = []

        #2. Initialize an empty stack (DFS) or queue (BFS). Add vi to the stack/queue.
        stack = [v_start]

        #3. If the stack/queue is not empty, pop/dequeue a vertex v.
        while len(stack) > 0:
            v = stack.pop()

        #4. Perform any desired processing on v.
            if v == v_end:
                visited.append(v)
                return visited

        # (DFS only): If v is not in the set of visited vertices:
            if v not in visited:

            # Add v to the set of visited vertices.
                visited.append(v)

        # Push each vertex that is direct successor of v to the stack.
                level = []
                for successor in self.adj_list[v]:
                    level.append(successor)
                level.sort(reverse=True)
                for i in level:
                    stack.append(i)

            #7. Repeat from 3, if while loop still active.

        # Otherwise, return visited
        return visited


    def bfs(self, v_start, v_end=None) -> []:
        """
        Take v_start (required) and v_end (optional). Beginning at v_start,
        perform breadth-first search (BFS) in the graph and return list of
        vertices visited during BFS, in the order visited.
        """

        # If the starting vertex is not in the graph, return empty list
        if v_start not in self.adj_list:
            return []

        # Initialize empty set of visited vertices.
        visited = []

        # Initialize empty queue. Add v_start to the queue.
        queue = [v_start]

        # If the queue is not empty, dequeue a vertex, v.
        while len(queue) > 0:
            v = queue[0]
            del queue[0]

        # Check if v matches optional endpoint, v_end.
            if v == v_end:
                visited.append(v)
                return visited

            # Add v to the set of visited vertices.
            if v not in visited:
                visited.append(v)

            # For each direct successor of v:
            level = []
            for successor in self.adj_list[v]:
                level.append(successor)
            level.sort()
            for i in level:

                # If successor is not in the set of visited vertices,
                # enqueue it into the queue
                if i not in visited:
                    queue.append(i)

        # Return visited when queue empty
        return visited


    def count_connected_components(self):
        """
        Return number of connected componets in the graph
        """

        # Declare list of subgraphs (lists), prepopulated with first subgraph
        visited = [self.bfs(list(self.adj_list.keys())[0])]

        # If length of first subgraph is same as entire graph, it's 1
        if len(visited[0]) == len(self.adj_list):
            return 1

        # Otherwise, there are > 1 connected components
        count = 2
        for vertex in self.adj_list.keys():
            for subgraph in visited:
                if vertex in subgraph:
                    break
                elif subgraph is visited[-1] and vertex not in subgraph:
                    visited.append(self.bfs(vertex))
                    count += 1
                    break

        # Return number of connected components
        return len(visited)


    def has_cycle(self):
        """
        Return True if graph (or distinct component) contains a cycle,
        False otherwise (acyclic graph).
        """

        #Start at a vertex
        vertices = list(self.adj_list.keys())

        #Start list of DFS searches for each vertex
        subgraph_list = [self.dfs(vertices[0])]

        #Create list of distinct subgraphs with >= 3
        for vertex in vertices:
            for subgraph in subgraph_list:
                if vertex in subgraph:
                    break
                elif subgraph is subgraph_list[-1] and vertex not in subgraph:
                    new_subgraph = self.dfs(vertex)
                    if len(new_subgraph) >= 3:
                        subgraph_list.append(new_subgraph)
                        break

        # For each distinct subgraph containing >= 3 vertices
        for subgraph in subgraph_list:
            if len(subgraph) >= 3 :

                # Declare starting vertex, visited dict, and edge stack
                v_start = subgraph[0]
                visited = {vertex: False for vertex in subgraph}
                visited[v_start] = True
                stack = [[None, v_start]]

                # Search graph until finding back edge indicating cycle
                while len(stack) > 0:
                    [predecessor, vertex] = stack.pop()

                    # Look at adjacent neighbors
                    for neighbor in self.adj_list[vertex]:

                        # Pass on visited edge
                        if neighbor is predecessor:
                            pass

                        # unvisited edge with visited node, cycle found
                        elif visited[neighbor] == True:
                            return True

                        # Otherwise, mark edge visited and push to edge stack
                        else:
                            visited[neighbor] = True
                            stack.append((vertex, neighbor))

        # no cycle found
        return False

# -----------------------BASIC TESTING--------------------------------- #
if __name__ == '__main__':

    print("\nPDF - method add_vertex() / add_edge example 1")
    print("----------------------------------------------")
    g = UndirectedGraph()
    print(g)

    for v in 'ABCDE':
        g.add_vertex(v)
    print(g)

    g.add_vertex('A')
    print(g)

    for u, v in ['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE', ('B', 'C')]:
        g.add_edge(u, v)
    print(g)


    print("\nPDF - method remove_edge() / remove_vertex example 1")
    print("----------------------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    g.remove_vertex('DOES NOT EXIST')
    g.remove_edge('A', 'B')
    g.remove_edge('X', 'B')
    print(g)
    g.remove_vertex('D')
    print(g)


    print("\nPDF - method get_vertices() / get_edges() example 1")
    print("---------------------------------------------------")
    g = UndirectedGraph()
    print(g.get_edges(), g.get_vertices(), sep='\n')
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE'])
    print(g.get_edges(), g.get_vertices(), sep='\n')


    print("\nPDF - method is_valid_path() example 1")
    print("--------------------------------------")
    g = UndirectedGraph(['AB', 'AC', 'BC', 'BD', 'CD', 'CE', 'DE'])
    test_cases = ['ABC', 'ADE', 'ECABDCBE', 'ACDECB', '', 'D', 'Z']
    for path in test_cases:
        print(list(path), g.is_valid_path(list(path)))


    print("\nPDF - method dfs() and bfs() example 1")
    print("--------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = 'ABCDEGH'
    for case in test_cases:
        print(f'{case} DFS:{g.dfs(case)} BFS:{g.bfs(case)}')
    print('-----')
    for i in range(1, len(test_cases)):
        v1, v2 = test_cases[i], test_cases[-1 - i]
        print(f'{v1}-{v2} DFS:{g.dfs(v1, v2)} BFS:{g.bfs(v1, v2)}')


    print("\nPDF - method count_connected_components() example 1")
    print("---------------------------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print(g.count_connected_components(), end=' ')
    print()


    print("\nPDF - method has_cycle() example 1")
    print("----------------------------------")
    edges = ['AE', 'AC', 'BE', 'CE', 'CD', 'CB', 'BD', 'ED', 'BH', 'QG', 'FG']
    g = UndirectedGraph(edges)
    test_cases = (
        'add QH', 'remove FG', 'remove GQ', 'remove HQ',
        'remove AE', 'remove CA', 'remove EB', 'remove CE', 'remove DE',
        'remove BC', 'add EA', 'add EF', 'add GQ', 'add AC', 'add DQ',
        'add EG', 'add QH', 'remove CD', 'remove BD', 'remove QG',
        'add FG', 'remove GE')
    for case in test_cases:
        command, edge = case.split()
        u, v = edge
        g.add_edge(u, v) if command == 'add' else g.remove_edge(u, v)
        print('{:<10}'.format(case), g.has_cycle())
