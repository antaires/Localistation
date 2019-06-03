//
//  main.cpp
//  Histogram
//
//  Created by Valia O'Donnell on 10/05/2019.
//  Copyright © 2019 Antaires. All rights reserved.
//
//  A program to take in a text file of waypoint data, determine all possible paths of
//  desired length, and render a histogram of distributions for unique paths
//  Handles both directed and undirected (for undirected, it will rotate BSD as needed)

#include <iostream>
#include <sstream> // for splitting the text string and storing to array
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include <unistd.h>
#include <iomanip> // for setw() print formatting
#include <stack> // for DFS
#include <cmath>
#include <map> // for storing unique BRDs and counting how often they occur

// adding to new github
#define DIRECTED false
#define BRD_LEN 80 // number of waypoints in BRD
#define TOTAL_WAYPOINTS 284 // total number of waypoints entered in text file
#define MAXLINELEN 100 // maximum length of waypoint data for total waypoints < 1000 (000 00 heading x y 000 000 000....?)
#define BSDLEN 4 // number of bits in a BSD
#define DATAFILE "/Users/valiaodonnell/Documents/School/Bristol/masterProject/histogram/histogram/data/undirected/doors_walls_FBLR/distance2/data_distance2_284_FBLR.txt"
//test datafile
//#define DATAFILE "/Users/valiaodonnell/Documents/School/Bristol/masterProject/histogram/histogram/testData3.txt"
#define OUTPUT "/Users/valiaodonnell/Documents/School/Bristol/masterProject/histogram/histogram/histogram_output/output.txt"
// don't change
#define STATS_ARR_LEN 102
#define ARRLEN TOTAL_WAYPOINTS + 1 // length of waypoint arr (total num of waypoints + 1) - each waypoint stored as its id

// FOR TESTING
// // id BSD(left,right) rotation posX posY connection_ids (variable length)
// std::string data = "1 11 -1 -1 -1\n2 10 1 -1 4\n3 01 2 -1 -1\n4 00 -1 -1 -1\n5 11 3 -1 -1\n6 01 5 -1 -1";

// ---------------------- //
// ------ NODE ---------- //
// ---------------------- //
class Node {
public:
    int bit;
    int count;
    Node* left;
    Node* right;
    
    Node(){
        count = 0;
        left = NULL;
        right = NULL;
        bit = -1;
    }
    
    void setBit(int b){
        bit = b;
    }
    
    int getBit(){
        return bit;
    }
};

// ---------------------- //
// ---- Binary Tree ----- //
// ---------------------- //
class BTree {
public:
    Node root;
    Node* current;
    int statsArray[STATS_ARR_LEN]; // stats array - track how many paths occur 1 - 100 times (lump all others together under 101)

    
    /*
     // the M is the capacity (8 = 8 bits)
     std::bitset<brdLen> brd(20); // bit sequence initialized to hold bit value of 20
     std::bitset<brdLen> brd(string("1100")); // initialized with string of specific bits
     
     // bits count from RIGHT to LEFT
     867543210
     so bit[1] is = 1 (above)
     
     // count fucntion returns number of set bits in bitset
     // size is total number of bits
     brd.test(0) // test returns 1 if bit is set else 0
     // set brd.set(pos, value) or brd.set(pos) // automaticallys sets it to 1
     
     // can convert decimal numbers to binary using bitset
     
     */
    BTree(){
        root = Node();      // root is first node
        current = &root;    // current is pointer
        initStatsArray();
    }
    
    void initStatsArray(){
        for(int i = 0; i<STATS_ARR_LEN; i++){
            statsArray[i] = 0;
        }
    }
    
    // used to add a full path to tree (tree built as paths added)
    // ends: count increased and returns to root
    void addBrd(std::string str_brd){
        // convert string to bit
        std::bitset<(BRD_LEN*BSDLEN)> brd(str_brd); // TODO potentially move this to Paths
        
        //resetToRoot();
        current = &root;
        
        //std::cout<<"path: " << str_brd<< std::endl;
        
        // for each bit:
        for (int i = (BRD_LEN*BSDLEN)-1; i >= 0; i--){
            //std::cout<<"i: "<<i<<" bit: "<<brd[i]<<std::endl;
            // add single bit to tree
            addBit(brd[i]);
        }
        increaseCurrentCount();
    }
    
    // used to add single bit to tree, uses 'current' node
    void addBit(int bit){
        // if bit 0, go left
        if (bit == 0){
            if (current->left == NULL){
                current->left = createNode(bit);
            }
            current = current->left;
        } else {
            if (current->right == NULL){
                current->right = createNode(bit);
            }
            current = current->right;
        }
    }
    
    Node* createNode(int bit){
        Node* n = new Node();
        n->setBit(bit);
        return n;
    }
    
    void increaseCurrentCount(){
        current->count++;
    }
    
    // go through each path of tree, build BRD and print count
    void print(){
        std::cout<<"PRINTING TREE: " << std::endl;
        std::string p = "";
        printTree(&root, p, BRD_LEN*BSDLEN);
        printStats();
    }
    
    void printTree(Node* n, std::string p, int cnt){
        
        /* NOT WORKING
         // this causes std::cout to write to OUTPUT file rather than terminal
         std::fstream file;
         file.open(OUTPUT, std::ios::out);
         std::string line;
         //backoup streambuffer of cout
         std::streambuf* stream_buffer_cout = std::cout.rdbuf();
         //std::streambuf* stream_buffer_cin  = std::cin.rdbuf();
         //get stream buffer of file
         std::streambuf* stream_buffer_file = file.rdbuf();
         //redirect cout to file
         std::cout.rdbuf(stream_buffer_file);
         */
        
        // add bit to path, except for root
        if (n->bit != -1){
            p = p + std::to_string(n->bit);
            cnt--;
        }
        
        if (cnt <= 0){
            std::cout << p << " - "<<std::setw(3)<<n->count<< " : ";
            for (int i = n->count; i > 0; i--){
                std::cout<<"\u25A0";
            }
            std::cout<<std::endl;
            // update stats info
            if (n->count <= 100){
                statsArray[n->count] = statsArray[n->count] + 1;
            } else {
                statsArray[STATS_ARR_LEN-1] = statsArray[STATS_ARR_LEN-1] + 1;
            }
            
        } else {
            if (n->left != NULL) {
                printTree(n->left, p, cnt);
            }
            if (n->right != NULL) {
                printTree(n->right, p, cnt);
            }
        }
        
        /*
         //redirect cout back to terminal
         std::cout.rdbuf(stream_buffer_cout);
         file.close();
         */
        
    }
    
    void printStats(){
        std::cout<<"\n"<<"STATS: "<<std::endl;
        for (int i = 0; i < STATS_ARR_LEN; i++){
            if (statsArray[i] != 0){
                std::cout<<"Count "<<std::setw(3)<<i<<" Occurred: "<<statsArray[i]<<std::endl;
            }
        }
        std::cout<<"Total paths: "<<sumStatsArray()<<std::endl;
        std::cout<<"Unique paths: "<<statsArray[1]<<std::endl;
    }
    
    int sumStatsArray(){
        int sum = 0;
        for(int i = 0; i < STATS_ARR_LEN; i++){
            sum += statsArray[i];
        }
        return sum;
    }
    
    // TODO
    void freeTree(){}
};

// ---------------------- //
// ------ Vector2d ------ //
// ---------------------- //
class Vector2d {
public:
    double x;
    double y;
    Vector2d(){
        x = 0;
        y = 0;
    }
};

// ---------------------- //
// ------ WAYPOINT ------ //
// ---------------------- //
class Waypoint {
public:
    // data members
    int id;
    std::string bsd;
    std::vector<int> conns;
    Vector2d pos;
    double rot; // this value comes from Unity in degrees, 0 - 360
    bool visited;
    Waypoint() {
        id = -1;
        bsd = "00";
        visited = false;
    }
    // functions
    void setId(int i){
        id = i;
    }
    void addConnection(int c){
        conns.push_back(c);
    }
    void print() {
        std::cout << "\nwaypoint id: " << id << std::endl;
        std::cout << "waypoint BSD: " << bsd << std::endl;
        for (int i = 0; i < conns.size(); i++){
            std::cout << conns[i] << " ";
        }
        std::cout << std::endl;
    }
};

// ---------------------- //
// --- WAYPOINT ARRAY --- //
// ---------------------- //
class WaypointArray {
public:
    // waypoints are stored in the array based on their id
    Waypoint waypointArray[ARRLEN];
    WaypointArray(){}
    void print(){
        // loop over waypoints and print (skips first = unused)
        for (int i = 1; i < ARRLEN; i++){
            waypointArray[i].print();
        }
    }
    void unvisitAll(){
        for (int i = 0; i < ARRLEN; i++){
            waypointArray[i].visited = false;
        }
        // set first to visited as it is NEVER used
        waypointArray[0].visited = true;
    }
};

// ---------------------- //
// --- data processing--- //
// ---------------------- //
// converts file to string, extracts line info, creates waypoints, and stores them to array
class DataProcessing {
public:
    std::string fileName;
    std::string line;
    std::vector<std::string> stringArray;
    
    DataProcessing (std::string fn){
        fileName = fn;
    }
    
    void process (std::string *p_data, WaypointArray *wa) {
        std::ifstream ifs; // open file in read mode
        ifs.open(fileName);
        std::string buf;
        
        // safety check
        if (!ifs){
            std::cout << "ERROR: cannot open file";
            exit(1);
        }
        
        //  ACCESS FILE CONTENTS
        getline(ifs, line);
        while (ifs) { // loop through file line by line
            int wordCount = 0;
            // ACCESS WORDS IN STRING
            buf = "";
            std::stringstream ss(line);
            // create vector of words
            Waypoint w;
            while(ss >> buf){
                if (wordCount == 0){
                    w.id = stringToInt(buf);
                } else if (wordCount == 1){
                    w.bsd = buf;
                    // TODO: access rotation and position of waypoint
                } else if (wordCount == 2){
                    w.rot = stringToDouble(buf);
                } else if (wordCount == 3){
                    w.pos.x = stringToDouble(buf);
                } else if (wordCount == 4){
                    w.pos.y = stringToDouble(buf);
                } else {
                    w.addConnection(stringToInt(buf));
                }
                wordCount++;
            }
            //w.print(); // TESTING prints each waypoint information
            
            // STORE NEW WAYPOINT IN ARRAY, INDEXED BY WAYPOINT ID
            wa->waypointArray[w.id] = w;
            
            getline(ifs, line);  // get next line if exists
        }
        
        ifs.close(); // close the file
    }
    
    int stringToInt(std::string str){
        std::stringstream stream(str);
        int x = 0;
        stream >> x;
        return x;
    }
    
    double stringToDouble(std::string str){
        std::istringstream buffer(str);
        double temp;
        buffer >> temp;
        return temp;
    }
};

// ---------------------- //
// ------ PATHS --------- //
// ---------------------- //
class Paths {
public:
    int len;
    int marker = 0;
    WaypointArray wa;
    BTree btree = BTree();
    //std::stack<Waypoint> stack; // stack for DFS TODO -- not needed if doing recursively
    
    Paths (int l, WaypointArray *warr){ // construct with desired length, waypoint array, and stats array
        len = l;
        wa = *warr;
    }
    
    void generatePaths(){ // input path length for BRDs
        // loop over graph: start with 1 because waypoint0 is not used
        std::string path = ""; // TODO make path a Bit sequence?
        for (int i = 1; i < ARRLEN; i++){
            wa.waypointArray[i].visited = true;
            buildPath(path, wa.waypointArray[i], wa.waypointArray[i].pos); // build all paths starting at each waypoint in turn
            wa.unvisitAll(); // clear "visited" bool to start next path
        }
        btree.print();
    }
    
    void buildPath(std::string path, Waypoint w, Vector2d prevPos){
        w.visited = true;
        // access w.pos for current position
        float heading = 0; // TODO UNITY: set lines to have 0 rotation
        
        // calculate heading for each new waypoint, and add w.bsd here in correct rotation
        heading = calculateHeading(prevPos, w.pos); // (prevPos, nextPos)
        
        if (DIRECTED){
            path = path + w.bsd;
        } else { // apply rotation
            path = path + rotateWaypoint(w, heading); // apply rotation -> not needed for directed graph
        }
        
        // check for connections if path length not reached yet
        if (path.length() < len * BSDLEN){
            // check every connection
            for(int i = 0; i < w.conns.size(); i++){
                // if connection has not been visited yet, visit it
                if (!(wa.waypointArray[w.conns[i]].visited)){
                    // need to pass in previous position to calculate heading
                    wa.waypointArray[w.conns[i]].visited = true; // prevent loops
                    buildPath(path, wa.waypointArray[w.conns[i]], w.pos); // recursive call to continue building path
                }
            }
        }
        
        // path has reached limit add to tree if long enough
        if (path.length() == len * BSDLEN){
            btree.addBrd(path);
        }
    }
    
    float calculateHeading(Vector2d prevPos, Vector2d nextPos){
        float x1 = prevPos.x;
        float y1 = prevPos.y;
        float x2 = nextPos.x;
        float y2 = nextPos.y;
        // calculates direction of travel between 2 points
        float theta_radians = atan2(y2 - y1, x2 - x1);
        // convert to degrees
        float theta_degrees = (theta_radians * M_PI) * 360.0 / (2.0 * M_PI);
        return theta_degrees;
    }
    
    std::string rotateWaypoint(Waypoint w, double heading){
        // if (nextWpHeading + heading is between (0 & 90) || (270 & 360) keep same BSD
        // else, switch (mirror) bsd (when between 91 * 269) switch <-- do this version
        
        double temp = w.rot + heading;
        if ( (temp > 90 + w.rot) && (temp < 270 + w.rot) ){ // + w.rot accounts for w.rot that is not 0
            //switch
            return switchBsd(w.bsd);
        }
        // TODO Unity: translate wp heading from -60 (+360) to keep positive values
        return w.bsd;
    }
    
    std::string switchBsd(std::string bsd){
        if (BSDLEN == 2){
            std::string newBsd = "";
            newBsd = bsd[1];
            newBsd = newBsd + bsd[0];
            return newBsd;
        } else if (BSDLEN == 6){ //TODO do this with bits, then convert to string it would be faster
            // assumes FB - L(d/w) - R(d/w)
            std::string newBsd = "";
            newBsd = bsd[1];
            newBsd = newBsd + bsd[0];
            newBsd = newBsd + bsd[4];
            newBsd = newBsd + bsd[5];
            newBsd = newBsd + bsd[2];
            newBsd = newBsd + bsd[3];
            return newBsd;
        } else if (BSDLEN == 4){
            // assumes FB LR
            std::string newBsd = "";
            newBsd = bsd[1];
            newBsd = newBsd + bsd[0];
            newBsd = newBsd + bsd[3];
            newBsd = newBsd + bsd[2];
            return newBsd;
        }
        std::cout<<"POTENTIAL ERROR: CHECK switchBsd() for correct BSD length"<<std::endl;
        return bsd;
    }
};

// ---------------------- //
// ------- MAIN --------- //
// ---------------------- //
int main(int argc, const char * argv[]) {
    
    /*
     // find current working directy
     char * dir = getcwd(NULL, 0);
     std::cout << dir << std::endl;
     */
    // print info for file
    std::cout<<"Number of waypoints: "<<BRD_LEN<<std::endl;
    std::cout<<"Number of bits in BRD: "<<BRD_LEN * BSDLEN <<std::endl;
    std::cout<<"Total number of waypoints: "<<TOTAL_WAYPOINTS<<std::endl;
    std::cout<<"data file: "<<DATAFILE<<std::endl<<std::endl;;
    
    // pick path length
    int pathlength = BRD_LEN; // number of waypoints per path
    std::string data;
    WaypointArray wa = WaypointArray(); // single waypoint array used throughout
    std::string fileName = DATAFILE;
    std::map<std::string, int> pathMap; // single map used throughout
    
    // generate string from file, builds waypoints from lines, stores them to waypoint array
    DataProcessing dp = DataProcessing(fileName);
    dp.process(&data, &wa);
    std::cout << "data processed to waypoint array" << std::endl;
    
    // generate paths
    Paths p = Paths(pathlength, &wa); // takes into account waypoint rotation and direction of travel
    p.generatePaths();
    //p.print();
    std::cout << "paths generated" << std::endl;
    
    return 0;
}
