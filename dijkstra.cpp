#include <iostream>
#include <vector>
#include <string>
#include <set>
#include <climits>
#include <algorithm>
#include<climits>
using namespace std;


class IEdge{
public:
    unsigned int first = 0; // first vertex
    unsigned int second = 0; //second vertex 
     ~IEdge(){}; //no edge weight if graph is unweighted
};

template <typename DistanceType, typename EdgeType> 
class IDistance{ //function distance, u have the path from a to b and edge from b to vertex c. You need to get a new distance from a to c
public:
    virtual DistanceType getNullDistance() = 0; //null distance
    virtual DistanceType getInfDistance() = 0;//inf distance
    virtual DistanceType getNewDistance(DistanceType dist, EdgeType* edge) = 0;//function
};

template <typename DistanceType, typename EdgeType>
class Graph{
    unsigned int VertexCapacity, EdgesCapacity;//number of vertexes, edges
public:
    vector<vector<EdgeType*>> edges; //list of ages
    IDistance<DistanceType, EdgeType>* distance;//distance func
    Graph(IDistance<DistanceType, EdgeType>* dist, unsigned int n): distance(dist), VertexCapacity(n),edges(vector<vector<EdgeType*>>(n)){}

    void addEdge(EdgeType* edge){
        ++EdgesCapacity;
        edges[edge->first].push_back(edge);
    }
    unsigned int getVertexCapacity(){
        return VertexCapacity;
    }
    unsigned int getEdgesCapacity(){
        return EdgesCapacity;
    }
};

template < typename DistanceType, typename EdgeType >
vector<DistanceType> dijkstra(Graph<DistanceType, EdgeType>* graph, unsigned int start){
    vector<DistanceType> dist(graph->getVertexCapacity(), graph->distance->getInfDistance());
    dist[start] = graph->distance->getNullDistance();
    set<pair<DistanceType, unsigned int>> setVertexes;
    setVertexes.insert(make_pair(dist[start], start));
    while (!setVertexes.empty()){
        unsigned int v = setVertexes.begin()->second;
        setVertexes.erase(setVertexes.begin());
        for (size_t i = 0; i < graph->edges[v].size(); ++i){
            size_t to = graph->edges[v][i]->second;
            DistanceType newDistance = graph->distance->getNewDistance(dist[v], graph->edges[v][i]);
            if (newDistance < dist[to])
            {
                setVertexes.erase(make_pair(dist[to], to));
                dist[to] = newDistance;
                setVertexes.insert(make_pair(dist[to], to));
            }
        }
    }
    return dist;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class NormalEdge: public IEdge{ //classic edge, 2 vertexes + weight
public:
    unsigned int first, second, weight;
    NormalEdge(unsigned int v1, unsigned int v2, unsigned int w): first(v1), second(v2), weight(w) {};
};

class NormalDistance: public IDistance<unsigned int, NormalEdge>{ // normal function: sum
public:
    unsigned int getNullDistance(){
        return 0;
    }
    unsigned int getInfDistance(){
        return INT_MAX;
    }
    unsigned int getNewDistance(unsigned int dist, NormalEdge* edge){
        return dist + edge->weight;
    }
    NormalDistance(){};
};


class BusEdge: public IEdge{ //edge for bus problem 
public:
    unsigned int first, second, start, finish;
    BusEdge(unsigned int v1, unsigned int v2, unsigned int st, unsigned int fi): first(v1), second(v2), start(st), finish(fi){};
};

class BusDistance: public IDistance<unsigned int, BusEdge>{
public:
    unsigned int getNullDistance(){
        return 0;
    }
    unsigned int getInfDistance(){
        return INT_MAX;
    }
    unsigned int getNewDistance(unsigned int dist, BusEdge* edge){
        return dist <= edge->start ? edge->finish : getInfDistance(); //func for bus problem
    }
    BusDistance(){};
};

class TransportEdge: public IEdge{ // edge for Transport Problem http://informatics.mccme.ru/mod/statements/view3.php?id=10845&chapterid=1967#1
public:
    unsigned int first, second, time, maxWeight;
    TransportEdge(unsigned int v1, unsigned int v2, unsigned int t, unsigned int mw): first(v1), second(v2), time(t), maxWeight(mw){};
};

class TransportDistance: public IDistance<unsigned int, TransportEdge>{
public:
    unsigned int weight;
    unsigned int getNullDistance(){
        return 0;
    }
    unsigned int getInfDistance(){
        return 1441;
    }
    unsigned int getNewDistance(unsigned int dist, TransportEdge* edge){
        return weight <= edge->maxWeight ? dist + edge->time : getInfDistance(); //func for transport problem
    }
    TransportDistance(){};
};

int main(){ //transport problem answer http://informatics.mccme.ru/mod/statements/view3.php?id=10845&chapterid=1967#1
    unsigned int n, m;
    cin >> n >> m;
    TransportDistance* transportDistance = new TransportDistance();
    Graph<unsigned int, TransportEdge>* graph = new Graph<unsigned int, TransportEdge>(transportDistance, n);
    for (size_t j = 0; j < m; ++j){
        unsigned int start, finish, time, maxWeight;
        cin >> start >> finish >> time >> maxWeight;
        start--;
        finish--;
        TransportEdge* newEdge1 = new TransportEdge(start, finish, time, maxWeight);
        TransportEdge* newEdge2 = new TransportEdge(finish, start, time, maxWeight);
        graph->addEdge(newEdge1);
        graph->addEdge(newEdge2);
    }

    unsigned int left = 3000000;
    unsigned int right = 1000000001 + left;
    while (right - left > 1){
        unsigned int middle = (left + right) / 2;
        transportDistance->weight = middle;
        vector<unsigned int> answer = dijkstra(graph, 0);
        bool ok = (answer[n - 1] < graph->distance->getInfDistance());
        if (ok)
            left = middle;
        else
            right = middle;
    }
    cout << (left - 3000000) / 100 << "\n";

    return 0;
}
