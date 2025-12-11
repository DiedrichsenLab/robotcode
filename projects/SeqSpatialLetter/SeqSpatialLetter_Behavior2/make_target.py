#!/usr/bin/env python3

import numpy as np
from collections import defaultdict
from random import choice

def has_eulerian_circuit(adj_matrix):
	"""
	Determine if an undirected graph has an Eulerian circuit.
	Returns (True/False, cycle_or_path_list)
	If no Eulerian circuit, cycle_list is None.
	If Eulerian circuit exists, cycle_list is a list of vertex indices in the cycle.
	"""
	has_circuit = True

	# Compute degrees (the number of edges from each node)
	n = len(adj_matrix)
	degrees = [sum(row) for row in adj_matrix]

	# Quick check: all degrees even
	if any(d % 2 != 0 for d in degrees):
		has_circuit = False

	# Find a starting vertex with non-zero degree
	start = None
	tmp = [i for i, d in enumerate(degrees) if d > 0]
	if not tmp is None:
		start = choice(tmp)
	if start is None:
		# No edges exist: trivially Eulerian circuit (empty cycle)
		has_circuit = True

	visited = [False]*n
	def DFS(u):
		visited[u] = True
		for v in range(n):
			if adj_matrix[u][v]>0 and not visited[v]:
				DFS(v)

	DFS(start)
	for i in range(n):
		if sum(adj_matrix[i])>0 and not visited[i]:
			has_circuit = False
	
	return has_circuit

def Eulerian_circuit(adj_matrix):
	n = len(adj_matrix)
	degrees = [sum(row) for row in adj_matrix]

	if not has_eulerian_circuit(adj_matrix):
		return None

	# Hierholzer's algorithm for undirected graphs
	# We'll work with a mutable copy of the adjacency matrix counts (edge multiplicity 1 or 0)
	g = [row[:] for row in adj_matrix]
	circuit = []

	tmp = [i for i, d in enumerate(degrees) if d > 0]
	start = choice(tmp)

	stack = [start]

	while stack:
		v = stack[-1]
		# v에서 나가는 사용 가능한 간선 탐색
		found = False
		for u in range(n):
			if g[v][u] > 0:
				# 간선 (v, u) 사용
				g[v][u] -= 1
				g[u][v] -= 1 # 무향이므로 대칭
				stack.append(u)
				found = True
				break
		if not found:
			# 더 이상 나갈 간선이 없으면 회로에 추가
			circuit.append(stack.pop())

	circuit.reverse() # 시작 순서대로

	return circuit

# Example usage:
adjA = np.ones((8,8)).astype(int)

circuit = Eulerian_circuit(adjA)
print("One Eulerian cycle (%d):"%(len(circuit)), circuit)
