The CDijkstraTransportationPlanner class is a routing system that calculates the shortest or fastest path between two locations using Dijkstra’s algorithm. It supports multiple transportation modes (walking, biking, bus) and generates step-by-step navigation instructions.

So in lines 1-19, we just established the header files that we needed to include. In SImplementation, we used different methods such as BuildGraph(), FormatLocation and CalculateDistance to aid in helping us implement the TransportationPlanner. Below is a sample usage of BuildGraph. Afterwards, we implemented the functions FindShortest, FindFastestPath, and GetPathDescription which were the most difficult for us to implement. Lastly, for the last few from the skeleton codwe to call on such as SortedNodeByIndex, FindFastestPath and PathDescription. 


void BuildGraph(){
        for (size_t i = 0; i < streetMap->WayCount(); ++i){
            auto way = streetMap->WayByIndex(i);
            if (!way || !IsWayTraversable(way)){
                continue; 
            } 

            for (size_t j = 0; j < way->NodeCount() - 1; ++j){
                TNodeID node1ID = way->GetNodeID(j); 
                TNodeID node2ID = way->GetNodeID(j+1);

                if (nodeToVertex.find(node1ID) == nodeToVertex.end() || 
                    nodeToVertex.find(node2ID) == nodeToVertex.end()){
                    continue; 
                } 

                TVertexID v1 = nodeToVertex[node1ID]; 
                TVertexID v2 = nodeToVertex[node2ID]; 

                auto node1 = GetNodeByID(node1ID); 
                auto node2 = GetNodeByID(node2ID); 

                if (!node1 || !node2){
                    continue; 
                }

                double distance = CalculateDistance(node1, node2); 

                bool oneWay = IsOneWay(way); 
                AddEdge(v1, v2, distance); 
                if (!oneWay){
                    AddEdge(v2, v1, distance); 
                }
            }
        }
    }




    
