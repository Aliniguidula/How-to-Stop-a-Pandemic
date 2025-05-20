import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

public class InformationSpread implements IInformationSpread {
    private Graph graph;
    private double tau;

    @Override
    public int loadGraphFromDataSet(String filePath, double tau) {
        this.tau = tau;
        int nodeCount = 0;
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String[] firstLine = br.readLine().split(" ");
            int totalNodes = Integer.parseInt(firstLine[0]);
            int totalEdges = Integer.parseInt(firstLine[1]);
            graph = new GraphL();
            graph.init(totalNodes + 1);
            String line;
            while ((line = br.readLine()) != null) {
                String[] parts = line.split(" ");
                int source = Integer.parseInt(parts[0]);
                int destination = Integer.parseInt(parts[1]);
                double weight = Double.parseDouble(parts[2]);
                if (weight >= tau) {
                    int intWeight = (int)(weight * 100);
                    graph.addEdge(source, destination, intWeight);
                    graph.addEdge(destination, source, intWeight);
                    if (graph.neighbors(source).length == 1) {
                        nodeCount++;
                    }
                    if (destination != 0 && graph.neighbors(destination).length == 1) {
                        nodeCount++;
                    }
                }
            }
        } catch (FileNotFoundException e) {
            System.err.println("File not found: " + filePath);
            return 0;
        } catch (IOException e) {
            System.err.println("Error reading file: " + e.getMessage());
            return 0;
        }
        return nodeCount;
    }

    @Override
    public int[] getNeighbors(int id) {
        if (graph == null || id < 0 || id >= graph.nodeCount()) {
            return new int[0];
        }
        return graph.neighbors(id);
    }

    @Override
    public List<Integer> path(int source, int destination) {
        if (graph == null ||
                source < 0 || source >= graph.nodeCount() ||
                destination < 0 || destination >= graph.nodeCount()) {
            return Collections.emptyList();
        }

        int n = graph.nodeCount();
        double[] dist = new double[n];
        int[] prev = new int[n];
        PriorityQueue<Node> pq =
                new PriorityQueue<>(Comparator.comparingDouble(node -> node.distance));

        Arrays.fill(dist, Double.POSITIVE_INFINITY);
        Arrays.fill(prev, -1);
        dist[source] = 0;
        pq.add(new Node(source, 0));

        while (!pq.isEmpty()) {
            Node current = pq.poll();
            int u = current.id;

            if (u == destination) {
                break;
            }
            if (current.distance > dist[u]) {
                continue;
            }

            for (int v : graph.neighbors(u)) {
                double originalWeight = graph.weight(u, v) / 100.0;
                double transformedWeight = -Math.log(originalWeight);

                double alt = dist[u] + transformedWeight;
                if (alt < dist[v]) {
                    dist[v] = alt;
                    prev[v] = u;
                    pq.add(new Node(v, alt));
                }
            }
        }

        if (prev[destination] == -1 && source != destination) {
            return Collections.emptyList();
        }

        LinkedList<Integer> path = new LinkedList<>();
        for (int at = destination; at != -1; at = prev[at]) {
            path.addFirst(at);
        }

        return path;
    }

    private static class Node {
        int id;
        double distance;

        Node(int id, double distance) {
            this.id = id;
            this.distance = distance;
        }
    }

    @Override
    public double avgDegree() {
        if (graph == null || graph.nodeCount() <= 1) {
            return 0.0;
        }

        int totalDegree = 0;
        int nodeCount = graph.nodeCount() - 1; // Count all nodes except 0

        for (int i = 1; i < graph.nodeCount(); i++) {
            totalDegree += graph.neighbors(i).length;
        }

        return nodeCount == 0 ? 0.0 : (double) totalDegree / nodeCount;
    }

    @Override
    public double rNumber() {
        return tau * avgDegree();
    }

    @Override
    public int generations(int seed, double threshold) {
        if (graph == null || seed < 1 || seed >= graph.nodeCount()) {
            return -1;
        }
        if (threshold < 0 || threshold > 1) {
            return -1;
        }
        if (threshold == 0) {
            return 0;
        }

        int totalNodes = graph.nodeCount() - 1;
        int targetNodes = (int) Math.ceil(threshold * totalNodes);
        if (targetNodes == 0) {
            return 0;
        }

        Queue<Integer> queue = new LinkedList<>();
        boolean[] visited = new boolean[graph.nodeCount()];
        int[] generations = new int[graph.nodeCount()];
        Arrays.fill(generations, -1);

        queue.add(seed);
        visited[seed] = true;
        generations[seed] = 0;
        int infectedCount = 1;
        int currentMaxGeneration = 0;

        if (infectedCount >= targetNodes) {
            return 0;
        }

        while (!queue.isEmpty()) {
            int current = queue.poll();

            for (int neighbor : graph.neighbors(current)) {
                if (!visited[neighbor] && neighbor != 0) {
                    visited[neighbor] = true;
                    generations[neighbor] = generations[current] + 1;
                    currentMaxGeneration = Math.max(currentMaxGeneration, generations[neighbor]);
                    infectedCount++;
                    queue.add(neighbor);

                    if (infectedCount >= targetNodes) {
                        return currentMaxGeneration;
                    }
                }
            }
        }

        return -1;
    }

    @Override
    public int degree(int n) {
        if (graph == null || n < 1 || n >= graph.nodeCount()) {
            return -1;
        }
        return graph.neighbors(n).length;
    }

    @Override
    public Collection<Integer> degreeNodes(int d) {
        List<Integer> nodes = new ArrayList<>();
        if (graph == null) {
            return nodes;
        }

        for (int i = 1; i < graph.nodeCount(); i++) {
            if (graph.neighbors(i).length == d) {
                nodes.add(i);
            }
        }
        return nodes;
    }

    @Override
    public int generationsDegree(int seed, double threshold, int d) {
        if (graph == null || seed < 1 || seed >= graph.nodeCount()) {
            return -1;
        }
        if (threshold < 0 || threshold > 1) {
            return -1;
        }
        if (degreeNodes(d).isEmpty()) {
            return -1;
        }
        if (threshold == 0) {
            return 0;
        }

        Graph immunizedGraph = copyGraph();
        immunizeDegree(immunizedGraph, d);

        if (immunizedGraph.neighbors(seed).length == 0) {
            return -1;
        }

        int totalNodes = graph.nodeCount() - 1;
        int targetNodes = (int) Math.ceil(threshold * totalNodes);
        if (targetNodes == 0) {
            return 0;
        }

        return computeGenerations(immunizedGraph, seed, targetNodes);
    }

    @Override
    public double rNumberDegree(int d) {
        if (graph == null || degreeNodes(d).isEmpty()) {
            return 0;
        }

        Graph immunizedGraph = copyGraph();
        immunizeDegree(immunizedGraph, d);

        double avgDegree = computeAvgDegree(immunizedGraph);
        return tau * avgDegree;
    }

    private Graph copyGraph() {
        Graph copy = new GraphL();
        copy.init(graph.nodeCount());

        for (int i = 0; i < graph.nodeCount(); i++) {
            copy.setValue(i, graph.getValue(i));
        }

        for (int i = 1; i < graph.nodeCount(); i++) {
            for (int neighbor : graph.neighbors(i)) {
                if (neighbor != 0) {
                    int weight = graph.weight(i, neighbor);
                    if (weight > 0) {
                        copy.addEdge(i, neighbor, weight);
                    }
                }
            }
        }
        return copy;
    }

    private void immunizeDegree(Graph g, int d) {
        for (int i = 1; i < g.nodeCount(); i++) {
            if (g.neighbors(i).length == d) {
                int[] neighbors = Arrays.copyOf(g.neighbors(i), g.neighbors(i).length);
                for (int neighbor : neighbors) {
                    g.removeEdge(i, neighbor);
                    g.removeEdge(neighbor, i);
                }
            }
        }
    }

    private double computeAvgDegree(Graph g) {
        if (g == null || g.nodeCount() <= 1) {
            return 0.0;
        }

        int totalDegree = 0;
        int nodeCount = g.nodeCount() - 1;

        for (int i = 1; i < g.nodeCount(); i++) {
            totalDegree += g.neighbors(i).length;
        }

        return nodeCount == 0 ? 0.0 : (double) totalDegree / nodeCount;
    }

    private int computeGenerations(Graph g, int seed, int targetNodes) {
        Queue<Integer> queue = new LinkedList<>();
        boolean[] visited = new boolean[g.nodeCount()];
        int[] generations = new int[g.nodeCount()];
        Arrays.fill(generations, -1);

        queue.add(seed);
        visited[seed] = true;
        generations[seed] = 0;
        int infectedCount = 1;
        int currentMaxGeneration = 0;

        if (infectedCount >= targetNodes) {
            return 0;
        }

        while (!queue.isEmpty()) {
            int current = queue.poll();

            for (int neighbor : g.neighbors(current)) {
                if (!visited[neighbor] && neighbor != 0) {
                    visited[neighbor] = true;
                    generations[neighbor] = generations[current] + 1;
                    currentMaxGeneration = Math.max(currentMaxGeneration, generations[neighbor]);
                    infectedCount++;
                    queue.add(neighbor);

                    if (infectedCount >= targetNodes) {
                        return currentMaxGeneration;
                    }
                }
            }
        }

        return -1;
    }

    @Override
    public double clustCoeff(int n) {
        if (graph == null || n < 1 || n >= graph.nodeCount()) {
            return -1;
        }

        int[] neighbors = graph.neighbors(n);
        int k = neighbors.length;

        if (k <= 1) {
            return 0.0;
        }

        int edgesBetweenNeighbors = 0;
        for (int i = 0; i < k; i++) {
            for (int j = i + 1; j < k; j++) {
                if (graph.hasEdge(neighbors[i], neighbors[j])) {
                    edgesBetweenNeighbors++;
                }
            }
        }

        int maxPossibleEdges = k * (k - 1) / 2;

        return (double) edgesBetweenNeighbors / maxPossibleEdges;
    }

    @Override
    public Collection<Integer> clustCoeffNodes(double low, double high) {
        List<Integer> nodes = new ArrayList<>();
        if (graph == null || low < 0 || high > 1 || low > high) {
            return nodes;
        }

        for (int i = 1; i < graph.nodeCount(); i++) {
            double cc = clustCoeff(i);
            if (IInformationSpread.inRange(cc, low, high, 0.01)) {
                nodes.add(i);
            }
        }
        return nodes;
    }

    @Override
    public int generationsCC(int seed, double threshold, double low, double high) {
        if (graph == null || seed < 1 || seed >= graph.nodeCount()) {
            return -1;
        }
        if (threshold < 0 || threshold > 1) {
            return -1;
        }
        if (clustCoeffNodes(low, high).isEmpty()) {
            return -1;
        }
        if (threshold == 0) {
            return 0;
        }

        Graph immunizedGraph = copyGraph();
        immunizeCC(immunizedGraph, low, high);

        if (immunizedGraph.neighbors(seed).length == 0) {
            return -1;
        }

        int totalNodes = graph.nodeCount() - 1;
        int targetNodes = (int) Math.ceil(threshold * totalNodes);
        if (targetNodes == 0) {
            return 0;
        }

        return computeGenerations(immunizedGraph, seed, targetNodes);
    }

    @Override
    public double rNumberCC(double low, double high) {
        if (graph == null || clustCoeffNodes(low, high).isEmpty()) {
            return 0;
        }

        Graph immunizedGraph = copyGraph();
        immunizeCC(immunizedGraph, low, high);

        double avgDegree = computeAvgDegree(immunizedGraph);
        return tau * avgDegree;
    }

    private void immunizeCC(Graph g, double low, double high) {
        for (int i = 1; i < g.nodeCount(); i++) {
            double cc = clustCoeff(i);
            if (IInformationSpread.inRange(cc, low, high, 0.01)) {
                int[] neighbors = Arrays.copyOf(g.neighbors(i), g.neighbors(i).length);
                for (int neighbor : neighbors) {
                    g.removeEdge(i, neighbor);
                    g.removeEdge(neighbor, i);
                }
            }
        }
    }

    @Override
    public Collection<Integer> highDegLowCCNodes(int lowBoundDegree, double upBoundCC) {
        List<Integer> nodes = new ArrayList<>();
        if (graph == null || lowBoundDegree < 0 || upBoundCC < 0 || upBoundCC > 1) {
            return nodes;
        }

        for (int i = 1; i < graph.nodeCount(); i++) {
            int degree = graph.neighbors(i).length;
            double cc = clustCoeff(i);

            if (degree >= lowBoundDegree &&
                    (IInformationSpread.inRange(cc, 0, upBoundCC, 0.01)
                            || cc <= upBoundCC)) {
                nodes.add(i);
            }
        }
        return nodes;
    }

    @Override
    public int generationsHighDegLowCC(int seed, double threshold,
                                       int lowBoundDegree, double upBoundCC) {
        if (graph == null || seed < 1 || seed >= graph.nodeCount()) {
            return -1;
        }
        if (threshold < 0 || threshold > 1) {
            return -1;
        }
        if (highDegLowCCNodes(lowBoundDegree, upBoundCC).isEmpty()) {
            return -1;
        }
        if (threshold == 0) {
            return 0;
        }

        Graph immunizedGraph = copyGraph();
        immunizeHighDegLowCC(immunizedGraph, lowBoundDegree, upBoundCC);

        if (immunizedGraph.neighbors(seed).length == 0) {
            return -1;
        }

        int totalNodes = graph.nodeCount() - 1;
        int targetNodes = (int) Math.ceil(threshold * totalNodes);
        if (targetNodes == 0) {
            return 0;
        }

        return computeGenerations(immunizedGraph, seed, targetNodes);
    }

    @Override
    public double rNumberDegCC(int lowBoundDegree, double upBoundCC) {
        if (graph == null || highDegLowCCNodes(lowBoundDegree, upBoundCC).isEmpty()) {
            return 0;
        }

        Graph immunizedGraph = copyGraph();
        immunizeHighDegLowCC(immunizedGraph, lowBoundDegree, upBoundCC);

        double avgDegree = computeAvgDegree(immunizedGraph);
        return tau * avgDegree;
    }

    private void immunizeHighDegLowCC(Graph g, int lowBoundDegree, double upBoundCC) {
        for (int i = 1; i < g.nodeCount(); i++) {
            int degree = g.neighbors(i).length;
            double cc = clustCoeff(i);

            if (degree >= lowBoundDegree &&
                    (IInformationSpread.inRange(cc, 0, upBoundCC, 0.01) || cc <= upBoundCC)) {
                int[] neighbors = Arrays.copyOf(g.neighbors(i), g.neighbors(i).length);
                for (int neighbor : neighbors) {
                    g.removeEdge(i, neighbor);
                    g.removeEdge(neighbor, i);
                }
            }
        }
    }
}