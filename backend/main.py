
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Tuple
import heapq
from itertools import combinations
from collections import defaultdict
from heapdict import heapdict

app = FastAPI()

# Configure CORS to allow requests from frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ------------------------- Data Models ------------------------- #
class coordinates(BaseModel):
    x: int
    y: int

class response(BaseModel):
    path: List[Tuple[int, int]]

class allNodes(BaseModel):
    houseNodes: List[List[int]]
    gasNodes: List[List[int]]
    wallNodes: List[List[int]]
    grid_size: List[int]

# ------------------------- Pathfinding Algorithms ------------------------- #

def H_score(start,end):
    return ((((end[0] -start[0])**2) + ((end[1]-start[1])**2))** 0.5)*10

def G_score(parent, current, prevScore):
    if parent[0]!=current[0] and parent[1]!=current[1]:
        return 14 + prevScore
    return 10 + prevScore

def aStar(start_pos,end_pos,obstacles,gridSize:List[int]):
    # Initialize the open list (priority queue) and closed list
    open_list = []
    closed_list = set()
    
    directions=[(-1,-1),(1,0),(-1,1),(1,-1),(-1,0),(1,1),(0,-1),(0,1)]
    prevG_score=defaultdict(int)
    parent={}
    path=[]
    
    #-----------------------------------------------------------------------------#
    heapq.heappush(open_list,(H_score(start_pos,end_pos),start_pos[0],start_pos[1]))
    parent[start_pos]=None
    prevG_score[start_pos]=0
    
    
    
    while open_list:
        _, x, y=heapq.heappop(open_list)
        closed_list.add((x,y))
        
        if (x,y)==end_pos:
            currentNode=(x,y)
            while currentNode!=start_pos:
                path.append(currentNode)
                currentNode=parent[currentNode]
            path.append(start_pos)
            return path
        
        
        for dx,dy in directions:
            nx,ny=x+dx,y+dy
            if 0<=nx<gridSize[0]  and 0<=ny<gridSize[1] and (nx,ny) not in closed_list and (nx,ny) not in obstacles:
                g_score=G_score((x,y),(nx,ny),prevG_score[(x,y)])
                h_score=H_score((nx,ny),end_pos)
                f_score=g_score+h_score
                
                if (nx,ny) not in prevG_score or g_score<prevG_score[(nx,ny)]:
                    parent[(nx,ny)]=(x,y)
                    prevG_score[(nx,ny)]=g_score
                    heapq.heappush(open_list,(f_score,nx,ny))      
    return []                    
                
                
def prims(edges,nodes):
   
    graph=defaultdict(list)
    parent=defaultdict(lambda:-1)
    
    
    for path in edges:
        if len(path) < 2: continue
        start, end = tuple(path[0]), tuple(path[-1])
        weight = len(path)
        graph[start].append((end[0], end[1], weight))
        graph[end].append((start[0], start[1], weight))
       
    
    
    pq=heapdict()
    
    
    flag=1
    for node in nodes:
        if flag==1:
            pq[tuple(node)]=0
            flag=0
        else:
            pq[tuple(node)]=525
    #-------------------------------------------
    
    while pq:
        key, priority = pq.popitem()
        
        
        for nx,ny,weight in graph[key]:
            
            if (nx, ny) in pq and weight<pq[(nx,ny)]:
                parent[(nx,ny)]=key;
                pq[(nx,ny)]=weight
        
    optimizePath=[]
    for Parent,Node in parent.items():
        for edge in edges:
            l=len(edge)
            if (edge[0]==Parent and edge[l-1]==Node) or (edge[l-1]==Parent and edge[0]==Node):
                optimizePath=optimizePath+edge
               
    return optimizePath
    
    
#------------------------------------------------------------------------------#
def manhattan_dist(x1, y1, x2, y2):
    return abs(x1 - x2) + abs(y1 - y2)

def assign_destinations(grid_size, sources, destinations, max_distance=10):
    assignment = {}  # destination -> source

    for dx, dy in destinations:
        min_dist = float('inf')
        closest_source = None
        for sx, sy in sources:
            d = manhattan_dist(dx, dy, sx, sy)
            if d < min_dist:
                min_dist = d
                closest_source = (sx, sy)
        if min_dist <= max_distance:
            assignment[(dx, dy)] = closest_source
        # Else: Not assigned

    

    return assignment
def group_by_source(assignment):
    grouped = defaultdict(list)

    for node, source in assignment.items():
        if node != source:  # Exclude source itself for now
            grouped[source].append(node)

    result = []
    for source, nodes in grouped.items():
        nodes.append(source)  # Append parent source at the end
        result.append(nodes)

    return result



#-------------------------------------------------------------------------------#
    
# ------------------------- API Endpoints ------------------------- #
path_steps = []    # Stores all A* paths between node pairs
optimize_steps = []  # Stores optimized MST path


# POST request 
@app.post("/post")
def generatePath(Nodes: allNodes):
    
    assignment = assign_destinations(Nodes.grid_size, Nodes.gasNodes, Nodes.houseNodes, max_distance=10)
    grouped_lists = group_by_source(assignment)

    print("\nGrouped Destination Lists by Source:")
    for group in grouped_lists:
        print(group)
    #-------------------------------------------------------------------------------#
    global path_steps
    global optimize_steps
    path_steps=[]
    # nodes=[]
    # nodes = Nodes.houseNodes + Nodes.gasNodes

    
    
    for nodes in grouped_lists:
        edges=[]
        for start, end in combinations(nodes,2):
            obstacles=set()
            obstacles = set(tuple(node) for node in nodes if node != start and node != end)
            obstacles.update(tuple(node) for node in Nodes.wallNodes)
            path=(aStar((start[0],start[1]),(end[0],end[1]),obstacles,Nodes.grid_size))
            print("path:",path)
            edges.append(path)
            path_steps=path_steps + path
            print("path_steps:",path_steps)
       
        optimize_steps=optimize_steps+prims(edges,nodes)
    
    return {"message": "success"}




@app.get("/show")
def responsePath() -> response:
    """Returns all raw paths found by A*"""
    return response(path=path_steps)

@app.get("/optimize")
def optimizePath() -> response:
    """Returns optimized path from Prim's MST"""
    return response(path=optimize_steps)












