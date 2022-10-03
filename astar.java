package ShortestPath;

import Graph.DirectedGraph;
import Graph.Edge;
import Graph.Vertex;
import Maps.Node;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * The Dijkstra class represents a generic Dijkstra/A* search algorithm.
 *
 * @param <V> generic vertex
 * @param <E> generic edge
 */
public class Dijkstra<V extends Vertex<V, E>, E extends Edge<V, E>> {

  private final Map<String, V> vertexMap;
  private final V sourceVertex;
  private final V destinationVertex;
  private final DirectedGraph<V, E> directedGraph;
  private final Map<V, Double> distancesMap;
  private final Set<V> visitedSet;
  private static final double RADIUS = 6371;

  /**
   * In the constructor we initialize all instance variables.
   *
   * @param sourceVertex      vertex to start search from
   * @param destinationVertex vertex to end search at
   * @param directedGraph     directed graph to perform search on
   * @param vertexMap         hashmap with all vertices present in directed graph
   */
  public Dijkstra(V sourceVertex, V destinationVertex,
                  DirectedGraph<V, E> directedGraph, Map<String, V> vertexMap) {
    this.sourceVertex = sourceVertex;
    this.destinationVertex = destinationVertex;
    this.directedGraph = directedGraph;
    this.vertexMap = vertexMap;
    this.distancesMap = new HashMap<>();
    this.visitedSet = new HashSet<>();
  }

  /**
   * fastSearch() is the actual dijkstra/A* search algorithm. It takes
   * in a boolean as an argument to determine if it should run dijkstra or A*
   * If true, it will run A*
   *
   * @param aStar to determine if it should run dijkstra or A*
   */
  public void fastSearch(boolean aStar) {
    PriorityQueue<Map.Entry<V, Double>> priorityQueue =
            new PriorityQueue<>((entryOne, entryTwo) -> Double.compare(
                    entryOne.getValue(), entryTwo.getValue())
            );
    for (V currVertex : vertexMap.values()) {
      if (currVertex.equals(sourceVertex)) {
        distancesMap.put(currVertex, 0.0);
      } else {
        distancesMap.put(currVertex, Double.POSITIVE_INFINITY);
      }
    }
    priorityQueue.add(new AbstractMap.SimpleEntry<V, Double>(sourceVertex, 0.0));
    while (!priorityQueue.isEmpty()) {
      V currVertex = priorityQueue.poll().getKey();
      visitedSet.add(currVertex);
      for (E currWay : directedGraph.getVertexConnections().get(currVertex)) {
        if (!visitedSet.contains(currWay.getDestinationVertex())) {
          double haversineHeuristic = 0.0;
          if (aStar) {
            haversineHeuristic = this.haversineCalculator(currVertex)
                    - this.haversineCalculator(currWay.getDestinationVertex());
          }
          double newEstimate =
                  distancesMap.get(currVertex) + currWay.getEdgeWeight() + haversineHeuristic;
          if (newEstimate < distancesMap.get(currWay.getDestinationVertex())) {
            distancesMap.put(currWay.getDestinationVertex(), newEstimate);
            parentsMap.put(currWay.getDestinationVertex(), currVertex);
            priorityQueue.add(
                    new AbstractMap.SimpleEntry<V, Double>(
                            currWay.getDestinationVertex(), newEstimate));
          }
        }
      }
    }
  }

  /**
   * haversineCalculator() calculates the haversine distance between the
   * destinationVertex and the vertex provided as a parameter.
   *
   * @param vertex vertex to calculate haversine distance from
   *
   * @return haversine distance
   */
  public double haversineCalculator(V vertex) {
    Node sourceNode = (Node) vertex;
    Node destinationNode = (Node) destinationVertex;
    double latOneRad = Math.toRadians(sourceNode.getNodeLatitude());
    double lonOneRad = Math.toRadians(sourceNode.getNodeLongitude());
    double latTwoRad = Math.toRadians(destinationNode.getNodeLatitude());
    double lonTwoRad = Math.toRadians(destinationNode.getNodeLongitude());
    double dLat = latTwoRad - latOneRad;
    double dLon = lonTwoRad - lonOneRad;
    double a =
            Math.pow(Math.sin(dLat / 2), 2)
                    + (Math.pow(Math.sin(dLon / 2), 2) * Math.cos(latOneRad) * Math.cos(latTwoRad));
    return 2 * RADIUS * Math.asin(Math.sqrt(a));
  }
}