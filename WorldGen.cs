using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Linq;
using Algorithms.Poisson;
using Algorithms.Delaunay;
using Geometry;
using Barycentric;
using System;
using UnityEngine.Experimental.AI;
using UnityEngine.XR;

public class GraphGen : MonoBehaviour
{

    public class MapPolygon
    {
        public Polygon polygon;

    }

    public class Node
    {
        public MapPolygon tile;

        public Node(Polygon polygon)
        {

            this.tile = new MapPolygon();
            this.tile.polygon = polygon;


        }
    }

        public static List<Polygon> prune(List<Polygon> polygonList)
        {
            // Polygons
            for (int i = 0; i < polygonList.Count - 1; i++)
            {
                for (int j = i + 1; j < polygonList.Count; j++)
                {
                    // Edges
                    for (int f = 0; f < polygonList[i].getEdges().Count; f++)
                    {
                        for(int k = 0; k < polygonList[j].getEdges().Count; k++)
                        {
                            if (polygonList[i].getEdges()[f] == polygonList[j].getEdges()[k])
                            {
                                polygonList[i].getEdges()[f] = polygonList[j].getEdges()[k];
                            }
                        }
                    }

                }
            }

            return polygonList;
        }


    public Dictionary<Polygon, List<Node>> createNodes(List<Polygon> polygons)
    {
        Dictionary<Polygon, List<Node>> hashMap = new Dictionary<Polygon, List<Node>>();

        foreach (Polygon polygon in polygons)
        {
            List<Line> edges = polygon.getEdges();

            foreach (Polygon otherPolygon in polygons)
            {
                if (otherPolygon == polygon)
                    continue;

                List<Line> otherEdges = otherPolygon.getEdges();

                foreach (Line edge in edges)
                {
                    foreach (Line otherEdge in otherEdges)
                    {
                        if (edge == otherEdge)
                        {
                            Node node = new Node(otherPolygon);

                            if (hashMap.ContainsKey(polygon) != true)
                            {
                                List<Node> nodes = new List<Node>();
                                nodes.Add(node);
                                hashMap.Add(polygon, nodes);
                                continue;
                            }
                            hashMap[polygon].Add(node);
                        }
                    }
                }
            }
        }

        return hashMap;
    }

    const float WIDTH = 50;
    const float HEIGHT = 50;
    const float SPACING = 3;
    private Dictionary<Polygon, List<Node>> Nodes;
    private List<Polygon> Meshes;


    // Start is called before the first frame update
    void Start()
    {
        PoissonDiskSampling pointsGenerator = new PoissonDiskSampling();
        List<Point> samples = pointsGenerator.Samples(WIDTH, HEIGHT, SPACING);
        List<Point> RefinedSamples = new List<Point>();
        foreach (Point point in samples)
        {
            if (point.toVector() != new Vector3(-1, -1, 0))
            {
                RefinedSamples.Add(point);
            }
            
        }

        DelaunayTriangulation triangulator = new DelaunayTriangulation(WIDTH, HEIGHT, RefinedSamples);
        List<Triangle> triangles = triangulator.getTriangles();
        Debug.Log(triangles.Count);


        DualMesh mesh = new DualMesh(triangles);
        List<Polygon> meshes = mesh.getMeshes();
        meshes = prune(meshes);
        Nodes = createNodes(meshes);
        this.Meshes = meshes;

        foreach (Polygon polygon in Meshes)
        { 
            polygon.render(.05f, Color.blue);
        }

        
        //Point test = new Point(5, 5, 0);
        //test.render(PrimitiveType.Sphere);

        //meshes[0].render(.05f, Color.green);
        //Nodes[meshes[0]][0].tile.polygon.render(.05f, Color.green);
        //Vector2 center = new Vector2(WIDTH/2, HEIGHT/2);

        Node prevNode;
        /*foreach (Polygon polygon in Meshes)
        {

            foreach (Node node in Nodes[polygon])
            {

                Line point1 = node.edge.edge.vertices.Item1;
                Line point2 = node.edge.edge.vertices.Item2;

                if (point1.z = 0)
                {
                    float nx = (point1.x/center.x) - 1; 
                    float ny = (point1.y/center.y) - 1;
                    float d = Mathf.Min(1, (Mathf.pow(nx, 2) + Mathf.pow(ny, 2)) / sqrt(2));
                    elevation = (elevation + (1 - d)) / 2;
                    point1.z = elevation;
                }
            }
        }*/



    }

        // Update is called once per frame
    void Update()
    {



    }

}
