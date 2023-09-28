// Useful to sort lists by a custom key
import java.util.Comparator;


/// In this file you will implement your navmesh and pathfinding.

/// This node representation is just a suggestion
class Node
{
  int id;
  ArrayList<Wall> polygon;
  PVector center;
  ArrayList<Node> neighbors;
  ArrayList<Wall> connections;
}

class Triangle
{
  PVector A;
  PVector B;
  PVector C;
  ArrayList<Triangle> neighbors;
  ArrayList<Wall> connections;
  PVector center;


  public Triangle (PVector A, PVector B, PVector C) {
    this.A = A;
    this.B = B;
    this.C = C;
    neighbors = new ArrayList<Triangle>();
    connections = new ArrayList<Wall>();
    center = PVector.mult(PVector.add(A, PVector.add(B, C)), 1.0/3.0);
  }
}

class FrontierEntry{
  Triangle triangle;
  float distance;
  float heuristic;
  FrontierEntry previous;
  
  public FrontierEntry (Triangle triangle, float distance, float heuristic, FrontierEntry previous){
     this.triangle = triangle;
     this.distance = distance;
     this.heuristic = heuristic;
     this.previous = previous;
  }
  
 
}

class FrontierCompare implements Comparator<FrontierEntry>{
  int compare(FrontierEntry A, FrontierEntry B){
    if (A.distance + A.heuristic < 
        B.distance + B.heuristic){
          return -1;
        }
    else if (A.distance + A.heuristic > 
        B.distance + B.heuristic){
          return 1;
        }  
    else { return 0;}
  }
  
}


class NavMesh
{
  ArrayList<Triangle> Triangles;
  ArrayList<Wall> NavLines;
  void bake(Map map)
  {

    NavLines = new ArrayList<Wall>();
    int size = map.walls.size();

    for (int k = 0; k < size; k++) {
      int index = k+1;
      if (index >= size) {
        index = 0;
      }
      if (PVector.dot(map.walls.get(k).normal, map.walls.get(index).direction) >= 0) { //dot product is a reflex


        for (int l = 0; l < size; l++) { //find connection
          PVector direction = PVector.sub(map.walls.get(k).end, map.walls.get(l).end).normalize();
          PVector tempStart = PVector.add(map.walls.get(l).end, PVector.mult(direction, 0.1));
          PVector tempEnd =  PVector.sub(map.walls.get(k).end, PVector.mult(direction, 0.1));
          if (!map.collides(tempStart, tempEnd) &&
            map.isReachable(PVector.mult(PVector.add(map.walls.get(k).end, map.walls.get(l).end), .5)))
          {
            stroke (255, 0, 0);
            Wall wallTester = new Wall(map.walls.get(k).end,
              map.walls.get(l).end);

            boolean canAdd = true;
            for (Wall W : NavLines) {
              if (W.crosses(tempStart, tempEnd)) {
                canAdd = false;
              }
            }
            if (canAdd == true) {
              NavLines.add(wallTester);
            }
          } else {

            //print ("Blocked");
          }
        }
      }
    }

    Triangles = new ArrayList<Triangle>();
    for (Wall W : map.outline) {
      ArrayList<PVector> neighborsA = new ArrayList<PVector>();
      neighborsA = getNeighbors(W.start);

      ArrayList<PVector> neighborsB = new ArrayList<PVector>();
      neighborsB = getNeighbors(W.end);


      for (PVector A : neighborsA) {
        for (PVector B : neighborsB) {
          if (A == B) {
            Triangles.add(new Triangle (W.start, W.end, A));
            
          }
        }
      }
    }
    
    for (Wall W : NavLines) {
      ArrayList<PVector> neighborsA = new ArrayList<PVector>();
      neighborsA = getNeighbors(W.start);

      ArrayList<PVector> neighborsB = new ArrayList<PVector>();
      neighborsB = getNeighbors(W.end);


      for (PVector A : neighborsA) {
        for (PVector B : neighborsB) {
          if (A == B) {
            Triangles.add(new Triangle (W.start, W.end, A));
            
          }
        }
      }
    }
    /*
    for (int i = 0; i < Triangles.size(); i++){
      print(" A: " + Triangles.get(i).A);
      print (" B: " +  Triangles.get(i).B);
      print (" C: " +  Triangles.get(i).C);
    }
    */
    
    for (Triangle A : Triangles) {
      for (Triangle B : Triangles) {
        
        
        if ((PVector.dist(A.A, B.A) < 0.01 && PVector.dist(A.B, B.B) < 0.01) || 
             (PVector.dist(A.A, B.A) < 0.01 && PVector.dist(A.B, B.C) < 0.01) ||
             (PVector.dist(A.A, B.B) < 0.01 && PVector.dist(A.B, B.C) < 0.01) ||
             (PVector.dist(A.A, B.A) < 0.01 && PVector.dist(A.C, B.B) < 0.01) ||
             (PVector.dist(A.A, B.A) < 0.01 && PVector.dist(A.C, B.C) < 0.01) ||
             (PVector.dist(A.A, B.B) < 0.01 && PVector.dist(A.C, B.C) < 0.01) ||
             (PVector.dist(A.B, B.A) < 0.01 && PVector.dist(A.C, B.B) < 0.01) ||
             (PVector.dist(A.B, B.A) < 0.01 && PVector.dist(A.C, B.C) < 0.01) ||
             (PVector.dist(A.B, B.B) < 0.01 && PVector.dist(A.C, B.C) < 0.01) ||
             (PVector.dist(A.B, B.B) < 0.01 && PVector.dist(A.A, B.A) < 0.01) ||
             (PVector.dist(A.B, B.C) < 0.01 && PVector.dist(A.A, B.A) < 0.01) ||
             (PVector.dist(A.B, B.C) < 0.01 && PVector.dist(A.A, B.B) < 0.01) ||
             (PVector.dist(A.C, B.B) < 0.01 && PVector.dist(A.A, B.A) < 0.01) ||
             (PVector.dist(A.C, B.C) < 0.01 && PVector.dist(A.A, B.A) < 0.01) ||
             (PVector.dist(A.C, B.C) < 0.01 && PVector.dist(A.A, B.B) < 0.01) ||
             (PVector.dist(A.C, B.B) < 0.01 && PVector.dist(A.B, B.A) < 0.01) ||
             (PVector.dist(A.C, B.C) < 0.01 && PVector.dist(A.B, B.A) < 0.01) ||
             (PVector.dist(A.C, B.C) < 0.01 && PVector.dist(A.B, B.B) < 0.01)){
               A.neighbors.add(B);
               B.neighbors.add(A);
         }
      }
    }
  }


  ArrayList<PVector> getNeighbors(PVector P)
  {
   // println ("Origin: " + P);
    ArrayList<PVector> neighbors = new ArrayList<PVector>();
    for (Wall X : map.outline) {
      if (PVector.dist(X.start, P) < 0.01) {
        neighbors.add(X.end);
      }
      if (PVector.dist(X.end, P) < 0.01) {
        neighbors.add(X.start);
      }
    }

    //ArrayList<PVector> neighborsB = new ArrayList<PVector>();
    for (Wall X : NavLines) {
      if (PVector.dist(X.start, P) < 0.01) {
        neighbors.add(X.end);
      }
      if (PVector.dist(X.end, P) < 0.01) {
        neighbors.add(X.start);
      }
      
      
    }
   
    return (neighbors);

  }
  
  ArrayList<PVector> findPath(PVector start, PVector destination)
  {
    ArrayList<FrontierEntry> searchFrontier = new ArrayList<FrontierEntry>();
    ArrayList<Triangle> removedTriangles = new ArrayList<Triangle>();
    ArrayList<Wall> TWalls = new ArrayList<Wall>();
    Triangle d = null;
    for (Triangle T: Triangles){
      Wall wallA = new Wall(T.A, T.B);
      Wall wallB = new Wall(T.A, T.C);
      Wall wallC = new Wall(T.B, T.C);
      TWalls.add(wallA);
      TWalls.add(wallB);
      TWalls.add(wallC);
      if(isPointInPolygon(start, TWalls) == true){
        searchFrontier.add(new FrontierEntry(T, 0, PVector.sub(T.center, destination).mag(), null));
        //TWalls.clear();
      }
      if(isPointInPolygon(destination, TWalls) == true){
        d = T;
      }
       TWalls.clear();

      
    }
      
    
    //int i = 0;
    //Triangle pretri;
    int counter = 0;
    int repeats;
    // infinite loop here; eliminate counter when solved
    while(d != searchFrontier.get(0).triangle)
    {
     Triangle t = searchFrontier.get(0).triangle;
     repeats = 0;
      if (removedTriangles != null)
      {
        for (Triangle R: removedTriangles)
        {
          if (t == R)
          {
            repeats++;
            searchFrontier.remove(0);
          }
        }
        //counter++;
        //println(counter);
      }
      if (repeats == 0)
      {
        removedTriangles.add(t);
       //Triangles.get(0).neighbors;
        for (Triangle T: t.neighbors)
        {
           searchFrontier.add(new FrontierEntry(T, searchFrontier.get(0).distance 
           +PVector.sub(t.center, T.center).mag(), 
             PVector.sub(T.center, destination).mag(), searchFrontier.get(0)));
        }
        searchFrontier.sort(new FrontierCompare());
        //counter++;
        //println(counter);
      }
      /*
     removedTriangles.add(t);
     //Triangles.get(0).neighbors;
      for (Triangle T: t.neighbors)
      {
         searchFrontier.add(new FrontierEntry(T, searchFrontier.get(0).distance 
         +PVector.sub(t.center, T.center).mag(), 
           PVector.sub(T.center, destination).mag(), searchFrontier.get(0)));
      }
      searchFrontier.sort(new FrontierCompare());
      counter++;
      println(counter);
      */
      //pretri = searchFrontier.get(0).previous;
    }
    
    ArrayList<PVector> tricenters = new ArrayList<PVector>();
    
    /*
    FrontierEntry current;
    current = searchFrontier.get(0);
    while (current.previous != null){
      tricenters.add(current.triangle.center);
      current = current.previous;
    }
    */
    
    FrontierEntry current, prev;
    PVector holder, holder2;
    current = searchFrontier.get(0);
    while (current.previous != null)
    {
      prev = current.previous;
      for (int i = 0; i < 3; i++)
      {
        if (i == 0) 
        { 
          holder = current.triangle.A;
          if (PVector.dist(holder, prev.triangle.A) < 0.01)
          {
            for (int j = 0; j < 2; j++)
            {
              if (j == 0) { holder2 = current.triangle.B;}
              else {holder2 = current.triangle.C;}
              if (PVector.dist(holder2, prev.triangle.B) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0));}
              else if (PVector.dist(holder2, prev.triangle.C) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0)); }
            }
          }
          else if (PVector.dist(holder, prev.triangle.B) < 0.01)
          {
            for (int j = 0; j < 2; j++)
            {
              if (j == 0) { holder2 = current.triangle.B;}
              else {holder2 = current.triangle.C;}
              if (PVector.dist(holder2, prev.triangle.A) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0));}
              else if (PVector.dist(holder2, prev.triangle.C) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0)); }
            }
          }
          else if (PVector.dist(holder, prev.triangle.C) < 0.01)
          {
            for (int j = 0; j < 2; j++)
            {
              if (j == 0) { holder2 = current.triangle.B;}
              else {holder2 = current.triangle.C;}
              if (PVector.dist(holder2, prev.triangle.B) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0));}
              else if (PVector.dist(holder2, prev.triangle.A) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0)); }
            }
          }
        }
        else if (i == 1) 
        { 
          holder = current.triangle.B;
          if (PVector.dist(holder, prev.triangle.A) < 0.01)
          {
            for (int j = 0; j < 2; j++)
            {
              if (j == 0) { holder2 = current.triangle.A;}
              else {holder2 = current.triangle.C;}
              if (PVector.dist(holder2, prev.triangle.B) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0)); }
              else if (PVector.dist(holder2, prev.triangle.C) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0));}
            }
          }
          else if (PVector.dist(holder, prev.triangle.B) < 0.01)
          {
            for (int j = 0; j < 2; j++)
            {
              if (j == 0) { holder2 = current.triangle.A;}
              else {holder2 = current.triangle.C;}
              if (PVector.dist(holder2, prev.triangle.A) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0)); }
              else if (PVector.dist(holder2, prev.triangle.C) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0)); }
            }
          }
          else if (PVector.dist(holder, prev.triangle.C) < 0.01)
          {
            for (int j = 0; j < 2; j++)
            {
              if (j == 0) { holder2 = current.triangle.A;}
              else {holder2 = current.triangle.C;}
              if (PVector.dist(holder2, prev.triangle.B) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0)); }
              else if (PVector.dist(holder2, prev.triangle.A) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0)); }
            }
          }
        }
        else
        { 
          holder = current.triangle.C;
          if (PVector.dist(holder, prev.triangle.A) < 0.01)
          {
            for (int j = 0; j < 2; j++)
            {
              if (j == 0) { holder2 = current.triangle.B;}
              else {holder2 = current.triangle.A;}
              if (PVector.dist(holder2, prev.triangle.B) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0)); }
              else if (PVector.dist(holder2, prev.triangle.C) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0)); }
            }
          }
          else if (PVector.dist(holder, prev.triangle.B) < 0.01)
          {
            for (int j = 0; j < 2; j++)
            {
              if (j == 0) { holder2 = current.triangle.B;}
              else {holder2 = current.triangle.A;}
              if (PVector.dist(holder2, prev.triangle.A) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0)); }
              else if (PVector.dist(holder2, prev.triangle.C) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0));}
            }
          }
          else if (PVector.dist(holder, prev.triangle.C) < 0.01)
          {
            for (int j = 0; j < 2; j++)
            {
              if (j == 0) { holder2 = current.triangle.B;}
              else {holder2 = current.triangle.A;}
              if (PVector.dist(holder2, prev.triangle.B) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0)); }
              else if (PVector.dist(holder2, prev.triangle.A) < 0.01) {tricenters.add(PVector.mult(PVector.add(holder, holder2), 1.0/2.0));}
            }
          }
        }
      }
      //tricenters.add(current.triangle.center);
      current = current.previous;
    }
    
    ArrayList<PVector> pathway = new ArrayList<PVector>();
    Wall edgepoint;
    PVector midEdgePoint;
    int count;
    
    //get lines between the centerpoints and add the midpoint of those lines
    /*
    PVector M;
    PVector N = tricenters.get(0);
    for (int i = tricenters.size()-1; i >= 0; i--){
      M = tricenters.get(i);
      //println(searchFrontier.get(0).triangle.center);
      //println(M);
      if (i == 0){
        edgepoint = new Wall(M, destination);
        for (Wall W: NavLines)
        {
          if (edgepoint.crosses(W.start, W.end))
          {
            midEdgePoint = PVector.mult(PVector.add(W.start, W.end), 1.0/2.0);
            pathway.add(midEdgePoint);
          }
        }
      }
      else{
        N = tricenters.get(i-1);
        
        edgepoint = new Wall(M, N);
        for (Wall W: NavLines)
        {
          if (edgepoint.crosses(W.start, W.end))
          {
            midEdgePoint = PVector.mult(PVector.add(W.start, W.end), 1.0/2.0);
            if (pathway != null)
            {
              if (pathway.size() > 0)
              {
                println("testing");
                count = 0;
                for (int j = 0; j < pathway.size(); j++)
                {
                  if (pathway.get(j) == midEdgePoint) { count++; }
                }
                if (count == 0) { pathway.add(midEdgePoint);}
              }
              else
              {
                //println(pathway.get(pathway.size() - 1));
                //println(midEdgePoint);
                pathway.add(midEdgePoint);
              }
            }
            break;
          }
        }
        //pathway.add(PVector.mult(PVector.add(M, N), 1.0/2.0));
      }
    }
    pathway.add(destination);
    */
    
    PVector sig;
    
    for (int i = tricenters.size() - 1; i >= 0; i--)
    {
      sig = tricenters.get(i);
      if (pathway.size() > 1)
      {
        if (sig != pathway.get(pathway.size() - 1))
        {
          pathway.add(tricenters.get(i));
        }
      }
      else {pathway.add(tricenters.get(i));}
    }
    pathway.add(destination);
    /*
    1. Create node array
    2. while (previous != null), add to Node array
    3. create a PVector pathway array
    4. add the midpoints between Node array to pathway array (for loop)
    5. Return pathway array (return statement)
    */
    
    /// implement A* to find a path
    ArrayList<PVector> result = null;
    ///*
    for (int i = 0; i < pathway.size(); i++)
    {
      println(pathway.get(i));
    }
    //*/
    
    return pathway;
  }


  void update(float dt)
  {
    draw();
  }

  void draw()
  {
   
    stroke(150);
    strokeWeight(3);

    for (Wall W : NavLines) {

      W.draw();
    }

    if (Triangles != null) {
      fill (255, 0, 0);
      for (Triangle T : Triangles) {
        stroke(250, 0, 0);
        circle(T.center.x, T.center.y, 10);
        for (Triangle S : T.neighbors) {
          line(T.center.x, T.center.y, S.center.x, S.center.y);
        }
      }
    }
  }
  
}
