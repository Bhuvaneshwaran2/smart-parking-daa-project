import java.util.*;

/**
 * Smart Parking Allocation and Navigation Optimization System
 * Educational DAA mini-project implementation.
 *
 * Input is a small in-memory road graph. Edge weights must be non-negative
 * when Dijkstra is used. This demo is not connected to sensors or a database.
 */
public class SmartParkingSystem {
    static class Edge {
        int to;
        double weight;
        Edge(int to, double weight) { this.to = to; this.weight = weight; }
    }
    static class Node {
        int id;
        double distance;
        Node(int id, double distance) { this.id = id; this.distance = distance; }
    }
    static class ParkingSpace {
        String id;
        int vertex;
        boolean available;
        String vehicleType; // "ANY", "CAR", "BIKE", etc.
        int priority;       // larger number means higher priority
        ParkingSpace(String id, int vertex, boolean available,
                     String vehicleType, int priority) {
            this.id=id; this.vertex=vertex; this.available=available;
            this.vehicleType=vehicleType; this.priority=priority;
        }
    }
    static class Allocation {
        ParkingSpace space;
        List<Integer> path;
        double distance;
        Allocation(ParkingSpace s, List<Integer> p, double d) {
            space=s; path=p; distance=d;
        }
    }

    private final List<List<Edge>> graph;
    private final List<ParkingSpace> spaces;
    private final int vertices;

    public SmartParkingSystem(int vertices) {
        if (vertices <= 0) throw new IllegalArgumentException("vertices must be positive");
        this.vertices = vertices;
        graph = new ArrayList<>();
        for (int i=0;i<vertices;i++) graph.add(new ArrayList<>());
        spaces = new ArrayList<>();
    }

    public void addRoad(int u, int v, double weight) {
        checkVertex(u); checkVertex(v);
        if (weight < 0 || Double.isNaN(weight) || Double.isInfinite(weight))
            throw new IllegalArgumentException("Road weight must be finite and non-negative");
        graph.get(u).add(new Edge(v, weight));
        graph.get(v).add(new Edge(u, weight)); // parking road network is undirected
    }
    public void addParkingSpace(ParkingSpace space) {
        checkVertex(space.vertex);
        spaces.add(space);
    }
    private void checkVertex(int v) {
        if (v < 0 || v >= vertices) throw new IllegalArgumentException("Invalid vertex: "+v);
    }

    /** Dijkstra: O((V+E) log V) with adjacency list and binary heap. */
    public Allocation allocateNearest(int entry, String vehicleType) {
        checkVertex(entry);
        double[] dist = new double[vertices];
        int[] prev = new int[vertices];
        Arrays.fill(dist, Double.POSITIVE_INFINITY);
        Arrays.fill(prev, -1);
        dist[entry]=0.0;
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingDouble(n -> n.distance));
        pq.add(new Node(entry, 0.0));
        while (!pq.isEmpty()) {
            Node cur=pq.poll();
            if (cur.distance > dist[cur.id]) continue;
            for (Edge e: graph.get(cur.id)) {
                double candidate=dist[cur.id]+e.weight;
                if (candidate < dist[e.to]) {
                    dist[e.to]=candidate;
                    prev[e.to]=cur.id;
                    pq.add(new Node(e.to, candidate));
                }
            }
        }

        ParkingSpace best=null;
        double bestScore=Double.POSITIVE_INFINITY;
        for (ParkingSpace s: spaces) {
            if (!s.available) continue;
            if (!"ANY".equalsIgnoreCase(s.vehicleType)
                    && !s.vehicleType.equalsIgnoreCase(vehicleType)) continue;
            if (Double.isInfinite(dist[s.vertex])) continue;
            // Primary objective: minimum route distance. Priority breaks exact distance ties.
            double score=dist[s.vertex];
            if (best==null || score < bestScore
                    || (Double.compare(score,bestScore)==0 && s.priority>best.priority)
                    || (Double.compare(score,bestScore)==0 && s.priority==best.priority
                        && s.id.compareTo(best.id)<0)) {
                best=s; bestScore=score;
            }
        }
        if (best==null) return null;
        List<Integer> path=new ArrayList<>();
        int at=best.vertex;
        while (at!=-1) {
            path.add(at);
            if (at==entry) break;
            at=prev[at];
        }
        if (path.get(path.size()-1)!=entry) return null;
        Collections.reverse(path);
        best.available=false; // reserve the space after successful allocation
        return new Allocation(best, path, bestScore);
    }

    public List<Integer> breadthFirstSearch(int start) {
        checkVertex(start);
        boolean[] seen=new boolean[vertices];
        List<Integer> order=new ArrayList<>();
        Queue<Integer> q=new ArrayDeque<>();
        seen[start]=true; q.add(start);
        while(!q.isEmpty()) {
            int u=q.remove(); order.add(u);
            for(Edge e:graph.get(u)) if(!seen[e.to]) {
                seen[e.to]=true; q.add(e.to);
            }
        }
        return order;
    }
    public List<Integer> depthFirstSearch(int start) {
        checkVertex(start);
        boolean[] seen=new boolean[vertices];
        List<Integer> order=new ArrayList<>();
        dfs(start, seen, order);
        return order;
    }
    private void dfs(int u, boolean[] seen, List<Integer> order) {
        seen[u]=true; order.add(u);
        for(Edge e:graph.get(u)) if(!seen[e.to]) dfs(e.to, seen, order);
    }

    public static int binarySearch(int[] sorted, int target) {
        int lo=0, hi=sorted.length-1;
        while(lo<=hi) {
            int mid=lo+(hi-lo)/2;
            if(sorted[mid]==target) return mid;
            if(sorted[mid]<target) lo=mid+1; else hi=mid-1;
        }
        return -1;
    }
    public static void mergeSort(int[] a) { mergeSort(a,0,a.length-1); }
    private static void mergeSort(int[] a,int l,int r) {
        if(l>=r) return;
        int m=l+(r-l)/2;
        mergeSort(a,l,m); mergeSort(a,m+1,r);
        int[] tmp=new int[r-l+1]; int i=l,j=m+1,k=0;
        while(i<=m && j<=r) tmp[k++]=a[i]<=a[j]?a[i++]:a[j++];
        while(i<=m) tmp[k++]=a[i++];
        while(j<=r) tmp[k++]=a[j++];
        System.arraycopy(tmp,0,a,l,tmp.length);
    }
    public static void quickSort(int[] a) { quickSort(a,0,a.length-1); }
    private static void quickSort(int[] a,int lo,int hi) {
        if(lo>=hi) return;
        int i=lo,j=hi,pivot=a[lo+(hi-lo)/2];
        while(i<=j) {
            while(a[i]<pivot) i++;
            while(a[j]>pivot) j--;
            if(i<=j) { int t=a[i]; a[i]=a[j]; a[j]=t; i++; j--; }
        }
        if(lo<j) quickSort(a,lo,j);
        if(i<hi) quickSort(a,i,hi);
    }
    public static int[] maxMin(int[] a) {
        if(a.length==0) throw new IllegalArgumentException("empty array");
        int min=a[0], max=a[0];
        for(int x:a) { if(x<min)min=x; if(x>max)max=x; }
        return new int[]{min,max};
    }
    public static int lcsLength(String a,String b) {
        int[] prev=new int[b.length()+1], cur=new int[b.length()+1];
        for(int i=1;i<=a.length();i++) {
            for(int j=1;j<=b.length();j++)
                cur[j]=(a.charAt(i-1)==b.charAt(j-1))?prev[j-1]+1:Math.max(prev[j],cur[j-1]);
            int[] t=prev; prev=cur; cur=t; Arrays.fill(cur,0);
        }
        return prev[b.length()];
    }
    public static int zeroOneKnapsack(int[] weight,int[] value,int capacity) {
        int[] dp=new int[capacity+1];
        for(int i=0;i<weight.length;i++)
            for(int c=capacity;c>=weight[i];c--)
                dp[c]=Math.max(dp[c],dp[c-weight[i]]+value[i]);
        return dp[capacity];
    }
    public static double fractionalKnapsack(int[] weight,int[] value,int capacity) {
        Integer[] idx=new Integer[weight.length];
        for(int i=0;i<idx.length;i++) idx[i]=i;
        Arrays.sort(idx,(i,j)->Double.compare((double)value[j]/weight[j],(double)value[i]/weight[i]));
        double total=0; int remaining=capacity;
        for(int i:idx) {
            if(weight[i]<=0) throw new IllegalArgumentException("weights must be positive");
            int take=Math.min(remaining,weight[i]);
            total+=(double)value[i]*take/weight[i];
            remaining-=take;
            if(remaining==0) break;
        }
        return total;
    }
    public static int kmpSearch(String text,String pattern) {
        if(pattern.isEmpty()) return 0;
        int[] lps=new int[pattern.length()];
        for(int i=1,len=0;i<pattern.length();) {
            if(pattern.charAt(i)==pattern.charAt(len)) lps[i++]=++len;
            else if(len>0) len=lps[len-1];
            else lps[i++]=0;
        }
        for(int i=0,j=0;i<text.length();) {
            if(text.charAt(i)==pattern.charAt(j)) { i++; j++; if(j==pattern.length()) return i-j; }
            else if(j>0) j=lps[j-1]; else i++;
        }
        return -1;
    }
    public static int rabinKarpSearch(String text,String pattern) {
        if(pattern.isEmpty()) return 0;
        if(pattern.length()>text.length()) return -1;
        final int base=256, mod=1_000_000_007;
        long p=0,t=0,pow=1;
        for(int i=0;i<pattern.length();i++) {
            p=(p*base+pattern.charAt(i))%mod;
            t=(t*base+text.charAt(i))%mod;
            if(i<pattern.length()-1) pow=(pow*base)%mod;
        }
        for(int i=0;i<=text.length()-pattern.length();i++) {
            if(p==t && text.regionMatches(i,pattern,0,pattern.length())) return i;
            if(i<text.length()-pattern.length()) {
                t=(t-text.charAt(i)*pow%mod+mod)%mod;
                t=(t*base+text.charAt(i+pattern.length()))%mod;
            }
        }
        return -1;
    }

    public static void main(String[] args) {
        SmartParkingSystem system=new SmartParkingSystem(6);
        system.addRoad(0,1,2);
        system.addRoad(1,2,2);
        system.addRoad(0,3,1);
        system.addRoad(3,4,2);
        system.addRoad(4,2,1);
        system.addRoad(2,5,2);
        system.addParkingSpace(new ParkingSpace("P-01",2,true,"CAR",1));
        system.addParkingSpace(new ParkingSpace("P-02",4,true,"ANY",2));
        system.addParkingSpace(new ParkingSpace("P-03",5,false,"ANY",3));

        Allocation result=system.allocateNearest(0,"CAR");
        if(result==null) {
            System.out.println("No eligible reachable parking space.");
        } else {
            System.out.println("Allocated space: "+result.space.id);
            System.out.println("Route: "+result.path);
            System.out.printf(Locale.US,"Distance: %.1f%n",result.distance);
        }
        System.out.println("BFS from entry: "+system.breadthFirstSearch(0));
        System.out.println("LCS length (PARKING, MARKING): "+lcsLength("PARKING","MARKING"));
        int[] values={9,3,7,1,5}; mergeSort(values);
        System.out.println("Merge-sorted values: "+Arrays.toString(values));
        System.out.println("KMP index of 'car' in 'smart car park': "+kmpSearch("smart car park","car"));
    }
}
