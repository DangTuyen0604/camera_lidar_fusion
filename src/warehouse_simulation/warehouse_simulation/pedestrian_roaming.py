"""Deterministic waypoint-graph roaming for warehouse pedestrians."""

from collections import deque
import random


class WaypointGraph:
    """Validated graph whose straight edges are known warehouse walkways."""

    def __init__(self, document):
        self.nodes = {
            str(name): (float(point[0]), float(point[1]))
            for name, point in document['nodes'].items()
        }
        self.neighbors = {name: [] for name in self.nodes}
        for edge in document['edges']:
            if len(edge) != 2 or edge[0] not in self.nodes or edge[1] not in self.nodes:
                raise ValueError(f'Invalid pedestrian graph edge: {edge}')
            left, right = edge
            if right not in self.neighbors[left]:
                self.neighbors[left].append(right)
            if left not in self.neighbors[right]:
                self.neighbors[right].append(left)
        if not self.nodes or any(not links for links in self.neighbors.values()):
            raise ValueError('Pedestrian graph must be non-empty and connected')
        origin = next(iter(self.nodes))
        if len(self._reachable(origin)) != len(self.nodes):
            raise ValueError('Pedestrian graph is disconnected')

    def _reachable(self, origin):
        seen = {origin}
        pending = [origin]
        while pending:
            node = pending.pop()
            for neighbor in self.neighbors[node]:
                if neighbor not in seen:
                    seen.add(neighbor)
                    pending.append(neighbor)
        return seen

    def nearest(self, point):
        """Return the closest graph node to an XY point."""
        x, y = point
        return min(
            self.nodes,
            key=lambda name: ((self.nodes[name][0] - x) ** 2 +
                              (self.nodes[name][1] - y) ** 2))

    def shortest_path(self, origin, destination):
        """Return an edge-safe shortest path including both endpoints."""
        pending = deque([origin])
        parent = {origin: None}
        while pending:
            node = pending.popleft()
            if node == destination:
                break
            for neighbor in self.neighbors[node]:
                if neighbor not in parent:
                    parent[neighbor] = node
                    pending.append(neighbor)
        if destination not in parent:
            raise ValueError(f'No pedestrian route from {origin} to {destination}')
        path = []
        node = destination
        while node is not None:
            path.append(node)
            node = parent[node]
        return list(reversed(path))


class RoamingAgent:
    """Independent reproducible destination, speed and pause selection."""

    def __init__(self, name, graph, seed, initial_xy, speed_range, pause_range):
        self.name = name
        self.graph = graph
        stable_name_seed = sum(
            (index + 1) * ord(character)
            for index, character in enumerate(name))
        self.rng = random.Random(int(seed) + stable_name_seed)
        self.current_node = graph.nearest(initial_xy)
        self.previous_origin = None
        self.destination = None
        self.route = []
        self.speed_range = tuple(float(value) for value in speed_range)
        self.pause_range = tuple(float(value) for value in pause_range)
        if (len(self.speed_range) != 2 or self.speed_range[0] <= 0.0 or
                self.speed_range[0] > self.speed_range[1]):
            raise ValueError(f'Invalid roaming speed range: {speed_range}')
        if (len(self.pause_range) != 2 or self.pause_range[0] < 0.0 or
                self.pause_range[0] > self.pause_range[1]):
            raise ValueError(f'Invalid roaming pause range: {pause_range}')
        self.speed = self.speed_range[0]
        self.pause_until = 0.0
        self.blocked_since = None
        self.completed_destinations = []

    def choose_trip(self):
        """Choose a non-repeating graph destination and route to it."""
        candidates = [
            node for node in self.graph.nodes
            if node != self.current_node and node != self.previous_origin
        ]
        if not candidates:
            candidates = [
                node for node in self.graph.nodes if node != self.current_node]
        self.previous_origin = self.current_node
        self.destination = self.rng.choice(candidates)
        self.route = self.graph.shortest_path(
            self.current_node, self.destination)[1:]
        self.speed = self.rng.uniform(*self.speed_range)
        return self.destination

    def arrive(self, now):
        """Record a completed trip and choose a reproducible random pause."""
        self.completed_destinations.append(self.destination)
        self.destination = None
        self.pause_until = now + self.rng.uniform(*self.pause_range)
        self.blocked_since = None

    def wait_or_retreat(self, now, retreat_after=2.0):
        """Wait briefly for traffic, then backtrack instead of deadlocking."""
        if self.blocked_since is None:
            self.blocked_since = now
            return False
        if now - self.blocked_since < retreat_after:
            return False
        self.destination = self.current_node
        self.route = [self.current_node]
        self.blocked_since = None
        return True
