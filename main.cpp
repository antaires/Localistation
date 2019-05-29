//
//  main.cpp
//  Histogram
//
//  Created by Valia O'Donnell on 10/05/2019.
//  Copyright © 2019 Antaires. All rights reserved.
//
//  A program to take in a text file of waypoint data, determine all possible paths of
//  desired length, and render a histogram of distributions for unique paths

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

// adding to new github - confirming works: branch: noRotation
//#define CONNSLEN 3 // length of connections array for Waypoint object
#define BRD_LEN 5
#define TOTAL_WAYPOINTS 142 // total number of waypoints entered in text file
#define MAXLINELEN 100 // maximum length of waypoint data for total waypoints < 1000 (000 00 heading x y 000 000 000....?)
//#define WPINFOLEN 5 // number of 'words' in the waypoint data: 000 00 000 000 000
//#define WPWORDLEN 3 // size of 'word' -- largest is 3 digits for total wp < 1000
#define BSDLEN 2 // number of bits in a BSD
#define DATAFILE "/Users/valiaodonnell/Documents/School/Bristol/masterProject/histogram/histogram/data_distance4_142.txt"
#define OUTPUT "/Users/valiaodonnell/Documents/School/Bristol/masterProject/histogram/histogram/histogram_output/output.txt"
// don't change
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
            // create node if it doesn't exist
            if (current->left == NULL){
                current->left = createNode(bit);
            }
            //createNodeIfNotExist(bit, current->left);
            current = current->left;
        } else {
            // create node if it doesn't exist
            if (current->right == NULL){
                current->right = createNode(bit);
            }
            //createNodeIfNotExist(bit, current->right); // pointer not pointing
            current = current->right;
        }
    }
    
    //void createNodeIfNotExist(int bit, Node** p){
    //    if (*p == NULL) {
    //        *p = createNode(bit);
    //        std::cout << "creating node with bit: " << bit << "p: "<<&p<<std::endl;
    //    }
    //}
    
    Node* createNode(int bit){
        //Node* n = (Node*) malloc(sizeof(Node));
        Node* n = new Node();
        n->setBit(bit);
        return n;
    }
    
    // increases visit count of 'current' node
    void increaseCurrentCount(){
        current->count++; // getting bad thread count here
    }
    
    // go through each path of tree, build BRD and print count
    void print(){
        std::cout<<"PRINTING TREE: " << std::endl;
        std::string p = "";
        printTree(&root, p, BRD_LEN*BSDLEN);
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
    std::vector<int> conns; // unknown number of connections
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
    //std::vector<Waypoint> waypointArray;
    Waypoint waypointArray[ARRLEN];
    WaypointArray(){}
    void print(){
        // loop over waypoints and print (skips first unused)
        for (int i = 1; i < ARRLEN; i++){
            waypointArray[i].print();
        }
    }
    void unvisitAll(){
        for (int i = 0; i < ARRLEN; i++){
            waypointArray[i].visited = false;
        }
        // set first to visited as it is unused
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
            
            // do I actually need to keep the string? TODO
            // *p_data = *p_data + line + "\n"; // process line
            
            getline(ifs, line);  // get next line if exists
            //std::cout << "Processing: " + data << std::endl;
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
    std::vector<std::string> pathsVector; //  unknown how many paths will be needed
    // int p; // tracking pathsArray location
    
    BTree btree = BTree();
    
    //std::stack<Waypoint> stack; // stack for DFS TODO -- not needed if doing recursively
    
    Paths (int l, WaypointArray *warr){ // construct with desired length and waypoint array
        len = l;
        wa = *warr;
    }
    
    void generatePaths(){ // input path length for BRDs
        // loop over graph: start with 1 because waypoint0 is not used
        
        // TODO make path a Bit sequence?
        std::string path = "";
        for (int i = 1; i < ARRLEN; i++){
            wa.waypointArray[i].visited = true;
            // TODO does it work that starting with its OWN position?
            buildPath(path, wa.waypointArray[i], wa.waypointArray[i].pos); // build all paths starting at each waypoint in turn
            wa.unvisitAll(); // clear "visited" bool to start next path
        }
        
        // TESTING
        btree.print();
    }
    
    void buildPath(std::string path, Waypoint w, Vector2d prevPos){
        w.visited = true;
        // access w.pos for current position
        float heading = 0; // TODO UNITY: set lines to have 0 rotation
        
        // calculate heading for each new waypoint, and add w.bsd here in correct rotation
        heading = calculateHeading(prevPos, w.pos); // (prevPos, nextPos)
        // can select only the headings I want here, return if heading not what I want...
        path = path + rotateWaypoint(w, heading); // apply rotation
        
        // check for connections if path length not reached yet
        if (path.length() < len * BSDLEN){
            // check every connection
            for(int i = 0; i < w.conns.size(); i++){
                // if connection has not been visited yet, visit it
                if (!(wa.waypointArray[w.conns[i]].visited)){
                    // need to pass in previous position to calculate heading
                    buildPath(path, wa.waypointArray[w.conns[i]], w.pos); // recursive call to continue building path
                }
            }
        }
        
        // path has reached limit add to array if long enough
        if (path.length() == len * BSDLEN){
            // TODO - tree idea: here is where I have the full path, this is where it should be
            // added to the tree, rather than pathsVector...
            // the toll on  mem will still be high because its recursive - can I do this iteratively with a stack?
            // alternately, this would be where count is increased in tree, and add bits as I generate them (after rotation
            // is calculated - so would be adding bits as 2-bit sequences to tree)
            
            // Testing without storage, shows that process can succeed, it will just take time
            //pathsVector.push_back(path);
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
        // If more semantic features are added to BDS, this will need to be updated
        std::string newBsd = "";
        newBsd = bsd[1];
        newBsd = newBsd + bsd[0];
        return newBsd;
    }
    
    std::vector<std::string> getPaths(){
        return pathsVector;
    }
    
    void print() {
        for(int i = 0; i < pathsVector.size(); i++){
            std::cout << pathsVector[i] << std::endl;
        }
    }
};

/*
 // ---------------------- //
 // ------ Datapoint ----- //
 // ---------------------- //
 // a class to contain a unique BRD and the count of how often it appears
 class Datapoint {
 public:
 long int count;
 std::string brd;
 Datapoint(std::string str){
 brd = str;
 count = 0;
 }
 void incrementCount(){
 count++;
 }
 void setCount(long int i){
 count = i;
 }
 long int getCount(){
 return count;
 }
 };*/

// ---------------------- //
// ------ Dataset ------- //
// ---------------------- //
// used to build a vector of unique Datapoints from all paths
class Dataset {
public:
    std::map<std::string, int> pathMap;
    //std::vector<Datapoint> dataset;
    std::vector<std::string> allPathsVector;
    Dataset(std::vector<std::string> apv, std::map<std::string, int> *map){
        allPathsVector = apv;
        pathMap = *map;
        
    }
    
    void buildSet() {
        // loop over all path BRDs in allPathsVector
        for (int i = 0; i < allPathsVector.size(); i++){
            // create an iterator for the map
            std::map<std::string, int>::iterator it = pathMap.find(allPathsVector[i]);
            // check if BRD exists in map
            if ( it != pathMap.end() ){
                // BRD is already in map -- access value from iterator and increment it
                it->second++;
            } else {
                // add BRD to map
                pathMap.insert(std::make_pair(allPathsVector[i], 1)); // starting with frequency = 1
            }
        }
    }
    
    /* OLD METHODS using vector
     void buildSet() { // TODO is this efficient? How can I optimize this process?
     // loop over all BRDs
     for(int i = 0; i < allPathsVector.size(); i++){
     // if BRD not already in set...
     if ( newBrd(allPathsVector[i]) ){
     // get frequency of each BRD
     long int freq = std::count(allPathsVector.begin(), allPathsVector.end(), allPathsVector[i]);
     // create datapoint
     Datapoint dp(allPathsVector[i]);
     dp.setCount(freq);
     // Add to dataSet
     dataset.push_back(dp);
     }
     }
     }
     // returns true if BRD not already in set
     bool newBrd(std::string brd){
     // loop over datapoints in dataset
     for (int i = 0; i < dataset.size(); i++){
     if (!dataset[i].brd.compare(brd)){
     return false;
     }
     }
     // returns true if no elements in dataset
     return true;
     }
     std::vector<Datapoint> getDataSet(){
     return dataset;
     }
     void print(){
     std::cout << "DATASET: " << std::endl;
     for(int i = 0; i < dataset.size(); i++){
     std::cout << "   BRD: " << dataset[i].brd << " Freq: " << dataset[i].getCount() << std::endl;
     }
     } // END OF OLD METHODS*/
    
    void print(int brdLen) {
        
        // main title and information
        std::cout << "\nHISTOGRAM GENERATOR" << std::endl;
        std::cout << "\nBRD length: " << brdLen << "\n" << std::endl;
        
        // create a map iterator and point to start of map
        std::map<std::string, int>::iterator it = pathMap.begin();
        // iterate over map using Iterator
        while (it != pathMap.end()){
            // access key (BRD)
            std::string brd = it->first;
            // access value (frequency)
            int freq = it->second;
            // print BRD (freq) -- setW(3) is for formatting number of connections
            std::cout << brd << " (" << std::setw(4) << freq << ") |";
            // loop over freq and add X
            for (int j = 0; j < freq; j++){
                std::cout << "\u25A0";
            }
            std::cout << std::endl;
            // increment iterator
            it++;
        }
    }
};

/*
 // ---------------------- //
 // ---- SIMPLE RENDER --- //
 // ---------------------- //
 // draws histogram to console
 class SimpleRender {
 public:
 std::map<std::string, int> pathMap;
 //std::vector<Datapoint> dataset;
 int brdLen;
 SimpleRender (std::map<std::string, int> *map, int blen){
 pathMap = *map;
 brdLen = blen;
 }
 
 void render() {
 // main title and information
 std::cout << "\nHISTOGRAM GENERATOR" << std::endl;
 std::cout << "\nBRD length: " << brdLen << "\n" << std::endl;
 
 // create a map iterator and point to start of map
 std::map<std::string, int>::iterator it = pathMap.begin();
 
 // iterate over map using Iterator
 while (it != pathMap.end()){
 // access key (BRD)
 std::string brd = it->first;
 // access value (frequency)
 int freq = it->second;
 
 // print BRD (freq) -- setW(3) is for formatting number of connections
 std::cout << brd << " (" << std::setw(4) << freq << ") |";
 // loop over freq and add X
 for (int j = 0; j < freq; j++){
 std::cout << "\u25A0";
 }
 std::cout << std::endl;
 
 // increment iterator
 it++;
 }
 
 }
 
 
 // TODO add more information about distribution etc
 };*/

// ---------------------- //
// ------- MAIN --------- //
// ---------------------- //
int main(int argc, const char * argv[]) {
    
    /*
     // find current working directy
     char * dir = getcwd(NULL, 0);
     std::cout << dir << std::endl;
     */
    
    // pick path length
    int pathlength = BRD_LEN; // number of waypoints per path
    std::string data;
    WaypointArray wa = WaypointArray(); // single waypoint array used throughout
    std::string fileName = DATAFILE;
    std::map<std::string, int> pathMap; // single map used throughout
    
    // generate string from file, builds waypoints from lines, stores them to waypoint array
    DataProcessing dp = DataProcessing(fileName);
    dp.process(&data, &wa);
    // TESTING DATA has been put into string
    //std::cout << data << std::endl;
    //wa.print();
    std::cout << "data processed to waypoint array" << std::endl;
    
    // generate paths
    Paths p = Paths(pathlength, &wa); // takes into account waypoint rotation and direction of travel
    p.generatePaths();
    //p.print();
    std::cout << "paths generated" << std::endl;
    
    // place paths onto a searchable tree structure OR hash table, and count how often it appears
    // make all paths into a set of BRDs, with a frequency count for each BRD
    Dataset ds = Dataset(p.getPaths(), &pathMap);
    ds.buildSet();
    ds.print(pathlength);
    
    /*
     // visualise unique paths + counts as histogram
     // take in a dataset, and render it!
     SimpleRender sr = SimpleRender(&pathMap, pathlength);
     sr.render();
     */
    
    return 0;
}
