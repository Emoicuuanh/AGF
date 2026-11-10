#!/usr/bin/env python
# -*- coding: utf-8 -*-
import heapq

def dijkstra(graph, start, end):
    heap = [(0, start, [])]
    visited = set()

    while heap:
        (cost, current, path) = heapq.heappop(heap)

        if current in visited:
            continue

        visited.add(current)

        path = path + [current]

        if current == end:
            return path, cost

        for neighbor, weight in graph[current]:
            heapq.heappush(heap, (cost + weight, neighbor, path))

    return None

def build_graph(data):
    graph = {}
    for node in data:
        neighbors = []
        for next_node in node.get('next', []):
            weight = calculate_distance(node, next_node, data)
            neighbors.append((next_node, weight))
        graph[node['name']] = neighbors
    return graph

def calculate_distance(node1, node2, data):
    pos1 = node1['data']['position']
    pos2 = next(n['data']['position'] for n in data if n['name'] == node2)
    return ((pos1['x'] - pos2['x'])**2 + (pos1['y'] - pos2['y'])**2)**0.5