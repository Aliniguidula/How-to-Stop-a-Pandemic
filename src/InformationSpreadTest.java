import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

import java.util.Arrays;
import java.util.Collection;
import java.util.List;

public class InformationSpreadTest {
    private InformationSpread infoSpread;
    private static final String TEST_FILE = "test_graph.mtx";
    private static final double TAU = 0.2;

    @Before
    public void setUp() {
        infoSpread = new InformationSpread();
        infoSpread.loadGraphFromDataSet(TEST_FILE, TAU);
    }

    @Test
    public void testLoadGraphFromDataSet() {
        int nodeCount = infoSpread.loadGraphFromDataSet(TEST_FILE, TAU);
        assertTrue(nodeCount > 0);
    }

    @Test
    public void testGetNeighbors() {
        int[] neighbors = infoSpread.getNeighbors(1);
        assertArrayEquals(new int[]{2, 3}, neighbors);

        neighbors = infoSpread.getNeighbors(6);
        assertEquals(0, neighbors.length);

        neighbors = infoSpread.getNeighbors(100);
        assertEquals(0, neighbors.length);
    }

    @Test
    public void testPath() {
        List<Integer> path = infoSpread.path(1, 12);
        if (!path.isEmpty()) {
            assertEquals(1, (int) path.get(0));
            assertEquals(12, (int) path.get(path.size() - 1));
        }

        path = infoSpread.path(1, 1);
        assertEquals(1, path.size());
        assertEquals(1, (int) path.get(0));

        path = infoSpread.path(1, 100);
        assertTrue(path.isEmpty());
    }

    @Test
    public void testAvgDegree() {
        double avgDegree = infoSpread.avgDegree();
        assertTrue(avgDegree > 0);
    }

    @Test
    public void testRNumber() {
        double rNumber = infoSpread.rNumber();
        assertTrue(rNumber >= 0);
    }

    @Test
    public void testGenerations() {
        int generations = infoSpread.generations(1, 0.5);
        assertTrue(generations >= 0);

        generations = infoSpread.generations(1, 0);
        assertEquals(0, generations);

        generations = infoSpread.generations(100, 0.5);
        assertEquals(-1, generations);
    }

    @Test
    public void testDegree() {
        int degree = infoSpread.degree(1);
        assertEquals(2, degree);

        degree = infoSpread.degree(100);
        assertEquals(-1, degree);
    }

    @Test
    public void testDegreeNodes() {
        Collection<Integer> nodes = infoSpread.degreeNodes(2);
        assertFalse(nodes.isEmpty());
        assertTrue(nodes.contains(1));

        nodes = infoSpread.degreeNodes(100);
        assertTrue(nodes.isEmpty());
    }

    @Test
    public void testGenerationsDegree() {
        int generations = infoSpread.generationsDegree(1, 0.5, 2);
        assertTrue(generations >= -1);

        generations = infoSpread.generationsDegree(1, 0, 2);
        assertEquals(0, generations);

        generations = infoSpread.generationsDegree(1, 0.5, 100);
        assertEquals(-1, generations);
    }

    @Test
    public void testRNumberDegree() {
        double rNumber = infoSpread.rNumberDegree(2);
        assertTrue(rNumber >= 0);

        rNumber = infoSpread.rNumberDegree(100);
        assertEquals(0, rNumber, 0.001);
    }

    @Test
    public void testClustCoeff() {
        double cc = infoSpread.clustCoeff(1);
        assertTrue(cc >= 0 && cc <= 1);

        cc = infoSpread.clustCoeff(100);
        assertEquals(-1, cc, 0.001);
    }

    @Test
    public void testClustCoeffNodes() {
        Collection<Integer> nodes = infoSpread.clustCoeffNodes(0.0, 1.0);
        assertFalse(nodes.isEmpty());

        nodes = infoSpread.clustCoeffNodes(1.1, 2.0);
        assertTrue(nodes.isEmpty());
    }

    @Test
    public void testGenerationsCC() {
        int generations = infoSpread.generationsCC(1, 0.5, 0.0, 1.0);
        assertTrue(generations >= -1);

        generations = infoSpread.generationsCC(1, 0, 0.0, 1.0);
        assertEquals(0, generations);

        generations = infoSpread.generationsCC(1, 0.5, 1.1, 2.0);
        assertEquals(-1, generations);
    }

    @Test
    public void testRNumberCC() {
        double rNumber = infoSpread.rNumberCC(0.0, 1.0);
        assertTrue(rNumber >= 0);

        rNumber = infoSpread.rNumberCC(1.1, 2.0);
        assertEquals(0, rNumber, 0.001);
    }

    @Test
    public void testHighDegLowCCNodes() {
        Collection<Integer> nodes = infoSpread.highDegLowCCNodes(2, 0.5);
        assertFalse(nodes.isEmpty());

        nodes = infoSpread.highDegLowCCNodes(100, 0.5);
        assertTrue(nodes.isEmpty());
    }

    @Test
    public void testGenerationsHighDegLowCC() {
        int generations = infoSpread.generationsHighDegLowCC(1, 0.5, 2, 0.5);
        assertTrue(generations >= -1);

        generations = infoSpread.generationsHighDegLowCC(1, 0, 2, 0.5);
        assertEquals(0, generations);

        generations = infoSpread.generationsHighDegLowCC(1, 0.5, 100, 0.5);
        assertEquals(-1, generations);
    }

    @Test
    public void testRNumberDegCC() {
        double rNumber = infoSpread.rNumberDegCC(2, 0.5);
        assertTrue(rNumber >= 0);

        rNumber = infoSpread.rNumberDegCC(100, 0.5);
        assertEquals(0, rNumber, 0.001);
    }

    @Test
    public void testStarGraph() {
        int nodes = infoSpread.loadGraphFromDataSet("star_graph.mtx", TAU);
        assertEquals(5, nodes);
        assertEquals(4, infoSpread.degree(1));
        assertEquals(1, infoSpread.degree(2));
        List<Integer> path = infoSpread.path(2, 5);
        assertEquals(Arrays.asList(2, 1, 5), path);
        assertEquals(0.0, infoSpread.clustCoeff(1), 0.001);
    }

    @Test
    public void testDisconnectedGraph() {
        int nodes = infoSpread.loadGraphFromDataSet("disconnected_graph.mtx", TAU);
        assertEquals(5, nodes);
        assertTrue(infoSpread.path(1, 4).isEmpty());
        assertEquals(0, infoSpread.degree(6));
        assertEquals(-1, infoSpread.generations(4, 0.75));
    }

    @Test
    public void testCommonFeatures() {
        infoSpread.loadGraphFromDataSet("star_graph.mtx", TAU);
        assertTrue(infoSpread.rNumber() > 0);
        Collection<Integer> hubs = infoSpread.highDegLowCCNodes(3, 0.1);
        assertTrue(hubs.contains(1));
    }

    private static final double DELTA = 0.0001;

    @Test
    public void testEmptyGraphLoading() {
        int loadedNodes = infoSpread.loadGraphFromDataSet("nothing_graph.mtx", 0.5);
        assertEquals(0, loadedNodes);
    }

    @Test
    public void testEmptyGraphOperations() {
        infoSpread.loadGraphFromDataSet("nothing_graph.mtx", 0.5);

        assertEquals(0.0, infoSpread.avgDegree(), DELTA);
        assertEquals(0.0, infoSpread.rNumber(), DELTA);
        assertTrue(infoSpread.path(1, 2).isEmpty());
        assertEquals(-1, infoSpread.generations(1, 0.5));
        assertEquals(-1, infoSpread.degree(1));
        assertTrue(infoSpread.degreeNodes(0).isEmpty());
        assertEquals(-1.0, infoSpread.clustCoeff(1), DELTA);
    }

    @Test
    public void testEmptyGraphImmunization() {
        infoSpread.loadGraphFromDataSet("nothing_graph.mtx", 0.5);

        assertEquals(0.0, infoSpread.rNumberDegree(2), DELTA);
        assertEquals(0.0, infoSpread.rNumberCC(0.1, 0.9), DELTA);
        assertEquals(-1, infoSpread.generationsDegree(1, 0.5, 2));
    }

    @Test
    public void testEmptyGraphNodeCollections() {
        infoSpread.loadGraphFromDataSet("nothing_graph.mtx", 0.5);

        assertTrue(infoSpread.degreeNodes(0).isEmpty());
        assertTrue(infoSpread.clustCoeffNodes(0.0, 1.0).isEmpty());
        assertTrue(infoSpread.highDegLowCCNodes(1, 0.5).isEmpty());
    }
}