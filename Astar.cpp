#include <bits/stdc++.h>

using namespace std;


#define Set(a, s) memset(a, s, sizeof (a))
#define rep(i, x, y) for(int i = x; i < y; i++)
#define Rep(i, x, y) for(int i = x; i <= y; i++)
#define vi vector<int>
#define vvi vector<vector<int> >
#define vp vector< pair< int, int > >
#define point pair<double, double >
#define pp push_back
#define mp make_pair
#define eps pow(10.0,-9.0)
#define MOD 1000000007
#define oo 1e18
#define Maxi 250000
typedef unsigned long long ull;
typedef long long ll;

int gcd( int a, int b ) { return ( b == 0 ? a : gcd( b, a%b )); }

const int row = 100, col = 100;

int opened[row+1][col+1], Map[row+1][col+1], dir_Map[row+1][col+1] ;
bool closed[row+1][col+1];

// direction : odd indices for moving diagonally, even ones for moving straight
const int xDir[] = { 1, 1, 0, 1, -1, -1, 0, -1};
const int yDir[] = { 0, 1, 1, -1, 0, -1, -1, 1};


struct Node
{
    // X, Y coordinates of the current node
    // dis_traveled distance traveled so far from the start point in the direction of the goal point
    // priority = distance traveled so far + estimation for the remaining distance
    int X, Y, dis_traveled, priority;
    
    //constructor
    Node( int x, int y, int l, int p) : X(x), Y(y), dis_traveled(l), priority(p) {}
    
    
    void Update_prt( const int& x, const int& y)
    {
        priority = dis_traveled + heuristic(x,y)*10;
    }

    void next_dis_traveled( const int &dir )
    {
        dis_traveled += ( (dir%2) ? 14 : 10 );// higher priority for moving straight
    }
    
    // the heuristic here is an estimation for the distance from current node to goal node
    int heuristic( const int &x, const int &y )
    {
        return  (int) ( sqrt( (x - X)*(x - X) + (y - Y)*(y - Y) ) ); // euclidean distance
    }

};

bool operator<(const Node & a, const Node & b)
{
  return a.priority > b.priority; //the greater the priority the less desirable is the node to be added
}

string A_star ( const int& xStart, const int& yStart, const int& xGoal, const int& yGoal )
{
    // Q : list of open nodes
    // temp : temporary queue to save certain nodes and return them back to Q
    priority_queue< Node > Q, temp;
    Node *n, *child;
    int x, y, newX, newY;
    
    // initially no node is visited yet
    Set(closed, false);
    Set(opened, 0);
    
    //create start node and push it in the queue and mark its position as opened one
    n = new Node( xStart, yStart, 0, 0);
    n->Update_prt( xGoal, yGoal);
    Q.push( (*n) );
    opened[ xStart ][ yStart ] = n->priority;

    while( !Q.empty() )
    {
        // get current node with highest priority and remove it from open set
        n = new Node( Q.top().X, Q.top().Y, Q.top().dis_traveled, Q.top().priority);
        Q.pop();
        x = n->X;
        y = n->Y;
        opened[x][y] = 0;
        closed[x][y] = 1;

        if( x == xGoal && y == yGoal ) // if the goal node is reached
        {
            string path = "";
            char c;
            int d;
            
            //trace back the path from the start to goal node
            while( !( x == xStart && y == yStart ) )
            {
                d = dir_Map[x][y];
                c = '0' + d;
                path += c;
                x -= xDir[ d ];
                y -= yDir[ d ];
            }

            delete n;
            reverse( path.begin(), path.end() );
            return path;
        }
        
        // check for new children for current node
        rep( i, 0, 8)
        {
            newX = x + xDir[i]; newY = y + yDir[i];
            // Map position = 1 if it has an obstacle
            if( newX >= 0 && newY >= 0 && newX < row && newY < col && Map[newX][newY] != 1 && !closed[newX][newY] )
            {
                child = new Node( newX, newY, n->dis_traveled, n->priority );
                child->next_dis_traveled(i);
                child->Update_prt(xGoal, yGoal);

                if( ! opened[ newX][newY] ) // if node not visited yet push it in open nodes
                {
                    opened[ newX][newY] = child->priority;
                    Q.push(*child);
                    dir_Map[newX][newY] = i;
                }
                else if ( opened[ newX][newY] > child->priority ) // if visited and a new path found with less cost update it
                {
                    opened[ newX][newY] = child->priority;
                    dir_Map[newX][newY] = i;

                    while( !( Q.top().X == newX && Q.top().Y == newY )) // save all nodes before child node
                    {
                        temp.push( Q.top() );
                        Q.pop();
                    }
                    Q.pop();
                    while( !temp.empty() ) // return the nodes back to open queue
                    {
                        Q.push(temp.top());
                        temp.pop();
                    }
                    Q.push(*child);
                }
                delete child; // for memory leakage
            }
        }
        delete n; // for memory leakage
    }

    return ""; // no path was found
}



int main()
{
    ios_base::sync_with_stdio(0);
    //freopen("input.in","r", stdin);
    //freopen("output.out","w", stdout);
	return 0;
}
