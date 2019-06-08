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
//
//  Handles corrupt paths and hamming distance comparison (ranks paths by distance)
//  handles L/R turn/no turn information

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
#define BRD_LEN 4 // number of waypoints in BRD
#define TOTAL_WAYPOINTS 7 // total number of waypoints entered in text file
#define MAXLINELEN 100 // maximum length of waypoint data for total waypoints < 1000 (000 00 heading x y 000 000 000....?)
#define BSDLEN 2 // number of bits in a BSD
//#define DATAFILE "/Users/valiaodonnell/Documents/School/Bristol/masterProject/histogram/histogram/data/undirected/doors_walls_FBLR/distance2/data_distance2_284_FBLR.txt"
//test datafile
#define DATAFILE "/Users/valiaodonnell/Documents/School/Bristol/masterProject/histogram/histogram/testData5.txt"
#define OUTPUT "/Users/valiaodonnell/Documents/School/Bristol/masterProject/histogram/histogram/histogram_output/output.txt"
// distance stats
#define LOWER_RANGE 0
#define UPPER_RANGE 10
#define RANK_STATS_SIZE 11

// Change this depending on turn/heading bits (un)used
// 0 = no turn info
// 1 = 1 bit turn info
// 2 = 2 bit turn info
#define EXTRA_BITS 2 //number of bits added to semantic BSD from turns, heading etc

// don't change
#define BSD_PLUS_EXTRA BSDLEN+EXTRA_BITS
#define BIT_SIZE BRD_LEN*BSDLEN + (BRD_LEN*EXTRA_BITS) // turn (2 bits - L/R)

// don't change
#define DIRECTED false
#define STATS_ARR_LEN 102
#define ARRLEN TOTAL_WAYPOINTS + 1 // length of waypoint arr (total num of waypoints + 1) - each waypoint stored as its id

// ---------------------- //
// --- SINGLE PATH ------ //
// ---------------------- /
class SinglePath {
public:
    std::bitset<(BIT_SIZE)> path;
    // TODO store turn path as separate path, or append it to end?
    int id;
    SinglePath(std::bitset<(BIT_SIZE)> brd, int waypointId){
        path = brd;
        id = waypointId;
    }
};

// ---------------------- //
// ------ NODE ---------- //
// ---------------------- //
class Node {
public:
    int bit;
    int count;
    std::vector <int> waypointIds; // list of waypoints that terminate at this node
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
    
    void addWaypointId(int id){
        waypointIds.push_back(id);
    }
    
    std::vector<int> getWaypointIds(){
        return waypointIds;
    }
    
    void printNode(){
        // print all contents of node to a single line, no new line after
        std::cout<<"||NODE bit:"<<bit<<" cnt:"<<count<<" ids:";
        for(int i = 0; i < waypointIds.size(); i++){
            std::cout<<waypointIds.at(i)<<" ";
        }
    }
};

// ---------------------- //
// ---- Binary Tree ----- //
// ---------------------- //
class BTree {
public:
    Node root;
    Node* current;
    std::vector<SinglePath> allPaths; // TODO potentially pass this as pointer to Paths class
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
    void addBrd(std::bitset<(BIT_SIZE)> brd, std::vector<int> waypointIds){
        int index = 0;
        
        //resetToRoot();
        current = &root;
        
        // for each bit:
        for (int i = (BIT_SIZE)-1; i >= 0; i--){ // not in reverse, but for binary, 0 starts on far right
            if (i % (BSD_PLUS_EXTRA) == 0){
                addBit(brd[i], waypointIds.at(index++));
            } else {
                addBit(brd[i], 0);
            }
        }
        increaseCurrentCount();
        
        // add brd to all paths
        SinglePath singlePath = SinglePath(brd, waypointIds.back());
        allPaths.push_back(singlePath);
    }
    
    // used to add single bit to tree, uses 'current' node
    void addBit(int bit, int waypointId){
        // if bit 0, go left
        if (bit == 0){
            if (current->left == NULL){
                current->left = createNode(bit);
            }
            if (waypointId != 0){
                current->left->addWaypointId(waypointId);
            }
            current = current->left;
        } else {
            if (current->right == NULL){
                current->right = createNode(bit);
            }
            if (waypointId != 0){
                current->right->addWaypointId(waypointId);
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
    
    std::vector<SinglePath> getAllPaths(){
        return allPaths;
    }
    
    // go through each path of tree, build BRD, print waypoint ids vector, print count
    void print(){
        //std::cout<<"\nPRINTING TREE: " << std::endl;
        //std::string p = "";
        //printTree(&root, p, BRD_LEN*BSDLEN + BRD_LEN);
        //printStats();
        //std::cout<<"\nPRINTING TEST TREE: " << std::endl;
        //printTestTree(&root);
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
            // loop over waypoint ids to print
            std::cout<<std::endl<<"wpId: ";
            std::vector<int> temp = n->getWaypointIds();
            for (int i = 0; i < temp.size(); i++){
                std::cout<<temp.at(i)<<" ";
            } std::cout<<std::endl;
    
            std::cout <<"path: "<< p << " --:";
            
            // print path count
            std::cout<<std::setw(3)<<n->count;
            // print square symbol for each occurance of path
            //for (int i = n->count; i > 0; i--){
            //    std::cout<<"\u25A0";
            //}
            
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
    
    void printTestTree(Node* n){
        if (n->left != NULL){
            n->left->printNode();
        }
        if (n->right != NULL){
            n->right->printNode();
        }
        std::cout<<std::endl;
        if (n->left != NULL) {
            printTestTree(n->left);
        }
        if (n->right != NULL) {
            printTestTree(n->right);
        }
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
        std::cout << "conns: ";
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
// --- DistanceStats ---- //
// ---------------------- //
class DistanceStats {
public:
    int distanceHist[BIT_SIZE] = {0}; //to occurance count distances
    // array of vectors to hold waypointIds that fall at each distance
    std::vector<int> waypointIds[BIT_SIZE];
    int correctId;
    int maxDistanceFrequency = 0;
    int rank = -1; // rank of correct id
    DistanceStats(){
        // initialize array of vectors
        for (int i=0; i < BIT_SIZE; i++){
            std::vector<int> row = std::vector<int>(20);
            waypointIds[i] = row;
        }
    }
    
    void reset(){
        for (int i=0; i < BIT_SIZE; i++){
            distanceHist[i] = 0;
            waypointIds[i].clear();
        }
    }
    
    void setId(int id){
        correctId = id;
    }
    
    void addDistance(int distanceIndex, int waypointId){
        distanceHist[distanceIndex] = distanceHist[distanceIndex] + 1;
        // add id to waypointId array at distanceIndex
        waypointIds[distanceIndex].push_back(waypointId);
        // track largest waypointId vector for printing purposes
        if (waypointIds[distanceIndex].size() > maxDistanceFrequency){
            maxDistanceFrequency = (int) waypointIds[distanceIndex].size();
        }
    }
    
    void setRank(){ // TODO how do I deal with TIES? - currently, rank is taken as lower rank of tie
        int rankCount = 0;
        int correctIdIndex = 0;
        
            for(int i=0; i< BIT_SIZE; i++){
                rankCount = distanceHist[i] + rankCount;
                for(int j=0; j <= maxDistanceFrequency; j++){
                if (waypointIds[i].size() > j){
                    if (waypointIds[i].at(j) == correctId){
                        correctIdIndex = i;
                        i = BIT_SIZE;
                        j = maxDistanceFrequency;
                    }
                }
            }
        }
        //std::cout<<"correctIdIndex: "<<correctIdIndex;
        //std::cout<<"\nrank:       : "<<rankCount<<std::endl;
        rank = rankCount;
    }
    
    int getRank(){
        return rank;
    }
    
    void printStats(){
        std::cout<<"\nDISTANCE STATS: "<<std::endl;
        // draw how many paths fall under each distance
        for(int i=LOWER_RANGE; i < UPPER_RANGE; i++){
            std::cout<<std::setw(5)<<distanceHist[i];
        } std::cout<<std::endl;
        // draw dividing line
        for(int i=LOWER_RANGE; i< UPPER_RANGE; i++){
            std::cout<<"-----";
        } std::cout<<std::endl;
        // draw distance value
        for(int i=LOWER_RANGE; i< UPPER_RANGE; i++){
            std::cout<<std::setw(5)<<i;
        } std::cout<<std::endl;
        // draw dividing line
        for(int i=LOWER_RANGE; i< UPPER_RANGE; i++){
            std::cout<<"^^^^^";
        } std::cout<<std::endl;
        // draw which waypoint id's appear at each distance value
        for(int j=0; j <= maxDistanceFrequency; j++){ // TODO: < or <=
            for(int i=LOWER_RANGE; i< UPPER_RANGE; i++){
                // draw waypoint id or if none, draw 3 spaces
                if (!waypointIds[i].empty() && waypointIds[i].size() > j){
                    if (waypointIds[i].at(j) == correctId){
                        std::cout<<"*!!!*";
                    } else {
                        std::cout<<std::setw(5)<<waypointIds[i].at(j);
                    }
                } else {
                    std::cout<<"    .";
                }
            } std::cout<<std::endl;
        }
    }
};

// ---------------------- //
// ------ CORRUPTPATH --- //
// ---------------------- //
class CorruptPath {
public:
    std::bitset<(BIT_SIZE)> corruptPath;
    std::vector<int> waypointIds;
    DistanceStats distanceStats = DistanceStats();
    bool isCorrupted;
    CorruptPath(){
        isCorrupted = false;
    }
    void setPath(std::bitset<(BIT_SIZE)> path, int correctId){
        distanceStats.setId(correctId);
        corruptPath = path;
        corrupt25();
    }
    void corrupt25(){
        // ASSUMES TURN BIT OCCURS AS FIRST BIT OF BSD
        // so for a 2 bit BSD, it would be: turnbit=x, bsd=yy: xyyxyyxyyxyy
        // and it will only corrupt the y's and skip over the x's
        int turnBitCount = 0;
        // loop over bits and flip 25% of the time (for total accuracy of 75%)
        for (int i = (BIT_SIZE)-1; i >= 0; i--){ // not in reverse, but for binary, 0 starst on far right
            // flips bit 25% of the time, but leaves turn bits as is
            if ( turnBitCount != 0 ){  // skips turn bits TODO - should these also be corrupted? accuracy of compass?
                if (flip25()){
                    corruptPath[i] = !corruptPath[i];
                }
            }
            turnBitCount++;
            turnBitCount = turnBitCount % (BSD_PLUS_EXTRA);
        }
        isCorrupted = true;
    }
    void hamming(std::bitset<(BIT_SIZE)> path, int id){
        // find hamming distance between corrupt path and input path, and store to DistanceStats
        int distance = 0;
        // compute hamming distance - loop over bits, and add 1 to distance for every different bit
        // use XOR so:
                // 0 ^ 0 = 0
                // 1 ^ 0 = 1
                // 0 ^ 1 = 1
                // 1 ^ 1 = 0
        distance = (int)(corruptPath ^ path).count();
        //for (int i = (BRD_LEN*BSDLEN)-1; i >= 0; i--){ // not in reverse, but for binary, 0 starst on far right
        //    if (corruptPath[i] ^ path[i]){
        //        distance++;
        //   }
        //}
        distanceStats.addDistance(distance, id);
        //std::cout<<"hamming distance: "<<distance<<std::endl;
    }
    
    void levenshtein(){
        // find levenshtein distance between corrupt path and input path, and store to DistanceStats
        // TODO
        // int distance = 0;
    }
    
    // probability function from www.geeksforgeeks.org/generate-0-1-25-75-probability
    int rand50(){
        // returns 0 50% of the time and 1 50% of the time
        // because rand() generates odd/even numbers in equal probability
        // and odd returns 1 and even will return 0
        return rand() & 1;
    }
    
    bool flip25(){
        // returns true 25% of the time
        if (rand50() | rand50()){
            return false;
        }
        return true;
    }
    
    int rand0_75(){
        // returns 0 75% of time, returns 1 25% of time
        return rand50() & rand50();
    }
    
    int rand1_75(){
        // returns 1 75% of time, returns 0 25% of time
        return rand50() | rand50();
    }
    
    void printPath(){
        std::cout<<"corrupted  :"<<corruptPath<<"\ncorrect id :"<<distanceStats.correctId<<std::endl;
    }
    
    void setRank(){
        distanceStats.setRank();
    }
    
    int getRank(){
        return distanceStats.getRank();
    }
    
    void reset(){
        distanceStats.reset();
        waypointIds.clear();
        isCorrupted = false;
    }
    
    void printStats(){
        distanceStats.printStats();
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
    CorruptPath cPath = CorruptPath();
    int rankHist[RANK_STATS_SIZE] = {0}; // 1-10 are true counts, 11 is everything else over 10
    
    Paths (int l, WaypointArray *warr){ // construct with desired length, waypoint array, and stats array
        len = l;
        wa = *warr;
    }
    
    void reset(){
        cPath.reset();
        marker = 0;
    }
    
    void generatePaths(){ // input path length for BRDs
        // loop over graph: start with 1 because waypoint0 is not used
        std::string path = ""; // TODO make path a Bit sequence?
        for (int i = 1; i < ARRLEN; i++){
            std::vector<int> waypointIds; // to track waypoint id's along path
            wa.waypointArray[i].visited = true;
            buildPath(path, waypointIds, wa.waypointArray[i], wa.waypointArray[i].pos, wa.waypointArray[i].rot, wa.waypointArray[i].rot); // build all paths starting at each waypoint in turn (starting 'prev rot' is just 0)
            wa.unvisitAll(); // clear "visited" bool to start next path
        }
        btree.print();
    }
    
    void buildPath(std::string path, std::vector<int> waypointIds, Waypoint w, Vector2d prevPos, double prevRot, double prevHeading){
        w.visited = true;
        // access w.pos for current position
        // calculate heading for each new waypoint, and add w.bsd here in correct rotation
        double heading;
        if (path == ""){ // for 1st wp in path, heading is its own rotation
            heading = w.rot;
        } else {
            heading = calculateHeading(prevPos, w.pos); // (prevPos, nextPos)
        }
        
        // add turn bit -- added BEFORE w BSD, since turn determined w/ prevPos to this wp
        // 2 bits (00=no turn, 10=left, 01=right, 11=not used)
        std::string turnBit;
        if (path.size() <= BSDLEN + 2){
            turnBit = "00"; // no turns for first 2 waypoints, as it will always be a straight line
        } else {
            //std::string turnBit = determineTurnBit(prevRot, w.rot);
            turnBit = determineTurnBit2(prevHeading, heading);
        }
        
        // rotate turnBit if needed (left --> right, etc)
        if (DIRECTED){
            path = path + turnBit + w.bsd;
        } else {
            //path = path + rotateTurn(turnBit, w.rot, heading) + rotateWaypoint(w, heading);
            // don't need to rotate turn bits, because calculation handles that already
            path = path + turnBit + rotateWaypoint(w, heading);
        }
        
        // add waypointId to vector of ids
        waypointIds.push_back(w.id);
        
        // check for connections if path length not reached yet
        if (path.length() < BIT_SIZE){
            // check every connection
            for(int i = 0; i < w.conns.size(); i++){
                // if connection has not been visited yet, visit it
                if (!(wa.waypointArray[w.conns[i]].visited)){
                    // need to pass in previous position to calculate heading
                    
                    wa.waypointArray[w.conns[i]].visited = true; // prevent loops
                    
                    // path, waypointIds, w, prevPos, prevRot, prevHeading)
                    buildPath(path, waypointIds, wa.waypointArray[w.conns[i]], w.pos, w.rot, heading); // recursive call to continue building path
                    wa.waypointArray[w.conns[i]].visited = false; // allows all connections to be attempted (1 connection may lead to multiple possible paths)
                }
            }
        }
        
        // path has reached limit add to tree if long enough
        if (path.length() == BIT_SIZE){
            // convert string to bit
            std::bitset<(BIT_SIZE)> brd(path); // account for addition of turnBits (BRD_LEN-1)
            btree.addBrd(brd, waypointIds);
            
            // TESTING
            std::cout<<"\nPATH ADDED:";
            std::cout<<"\n"<<brd<<std::endl;
            for(int i=0; i < waypointIds.size(); i++){
                std::cout<<"   "<<waypointIds.at(i);
            } std::cout<<std::endl;
            
            // TESTING
            testData5(path, waypointIds);
        }
    }
    
    void testData5(std::string path, std::vector<int> waypointIds){
        // convert waypoint ids to a string
        std::string ids = "";
        for(int i=0; i < waypointIds.size(); i++){
            ids = ids + std::to_string(waypointIds.at(i));
        }
        
        // check all paths
        // 1s
        if ( ids == "1234" ){
            assert( path == "0000000100100111");
        } else if ( ids == "1275" ){
            assert( path == "0000000101110001");
        } else if ( ids == "1657"){
            assert( path == "0000000010101011");
        } else if ( ids == "1654"){
            assert( path == "0000000010100011");
        }
        // 2s
        else if( ids == "2345" ){
            assert( path == "0001001001110101" );
        } else if ( ids == "2754" ){
            assert( path == "0001001100011011" );
        } else if ( ids == "2756" ){
            assert( path == "0001001100010100" );
        } else if ( ids == "2165" ){
            assert( path == "0001000010001010" );
        }
        // 3s
        else if( ids == "3457" ){
            assert( path == "0010001101010111" );
        } else if ( ids == "3456" ){
            assert( path == "0010001101010000");
        } else if ( ids == "3216" ){
            assert( path == "0010001000001000");
        } else if ( ids == "3275" ){
            assert( path == "0010001010110001");
        }
        // 4s
        else if( ids == "4572" ){
            assert( path == "0011000101110001" );
        } else if ( ids == "4561" ){
            assert( path == "0011000100000100");
        } else if ( ids == "4327" ){
            assert( path == "0011000110101011");
        } else if ( ids == "4321" ){
            assert( path == "0011000110100000");
        }
        // 5s
        else if( ids == "5612" ){
            assert( path == "0001000001000101" );
        } else if ( ids == "5721" ){
            assert( path == "0001001100011000"); // change if adding retro-active rotation
        } else if ( ids == "5723" ){
            assert( path == "0001001100010110");
        } else if ( ids == "5432" ){
            assert( path == "0001001110011010");
        }
        // 6s
        else if ( ids == "6572" ){
            assert( path == "0000001010110001" ); // rotate 2 if adding retr-active rotation
        } else if ( ids == "6543" ){
            assert( path == "0000001000111001");
        } else if ( ids == "6127" ){
            assert( path == "0000000001010111");
        } else if ( ids == "6123" ){
            assert( path == "0000000001010010");
        }
        // 7s
        else if ( ids == "7561" ){
            assert( path == "0011000101000100" ); // rotate 2 if adding retr-active rotation
        } else if ( ids == "7543" ){
            assert( path == "0011000110111001" ); // rotate 2 if adding retr-active rotation
        } else if ( ids == "7216" ){
            assert( path == "0011000110001000"); // rotate 2 if adding retr-active rotation
        } else if ( ids == "7234" ){
            assert( path == "0011000101100111"); 
        }
    }
    
    float calculateHeading(Vector2d prevPos, Vector2d nextPos){
        Vector2d square = Vector2d();
        square.x = (nextPos.x - prevPos.x);
        square.y = (nextPos.y - prevPos.y);
        double heading = atan2(square.y, square.x);
        // convert to degrees
        heading = (heading * M_PI) * 360.0 / (2.0 * M_PI);
        
        std::cout<<"\nP1("<<prevPos.x<<","<<prevPos.y<<")";
        std::cout<<"\nP2("<<nextPos.x<<","<<nextPos.y<<")";

        // was rotating opposite way than calculations expected,
        // so instead of 90 it was returning 282 (270)
        // and rather than 270 it was returning -287
        std::cout<<"\nHEADING pre adjust: "<<heading;
        if (heading == 0){
        } else if (heading < 0){
            heading = abs(heading);
        } else if (heading > 0 && heading < 360){
            heading = heading -180;
        } else {
            heading = heading - 360;
        }
        std::cout<<"\nHEADING post adjust: "<<heading;
         
         
        /* OLD VERSION
        float x1 = prevPos.x;
        float y1 = prevPos.y;
        float x2 = nextPos.x;
        float y2 = nextPos.y;
        // calculates direction of travel between 2 points
        float theta_radians = atan2(y2 - y1, x2 - x1);
        // convert to degrees
        float theta_degrees = (theta_radians * M_PI) * 360.0 / (2.0 * M_PI);
        return theta_degrees;
         */
        std::cout<<"\nheading: "<<heading;
        return heading;
    }
    
    std::string determineTurnBit2(double prevHeading, double heading){
        double h1 = prevHeading;
        double h2 = heading;
        
        // constrain
        h1 = constrain(h1);
        h2 = constrain(h2);
        
        double diff = h2 - h1;
        if (diff < 0){
            diff += 360;
        } else if (diff >= 360){
            diff -= 360;
        }
        
        std::cout<<"\nDetermine turn bit 2: ";
        std::cout<<"\nprevHeading: "<<prevHeading<<" h1:"<<h1;
        std::cout<<"\ncurrHeading: "<<heading<<" h2:"<<h2;
        std::cout<<"\ndiff: "<<diff;
        
        if (diff > 225 && diff < 315){
            // right turn
            return "01";
        } else if (diff > 45 && diff < 135){
            // left turn
            return "10";
        }
        return "00";
        
        /*
        // skip if one value 0
        double diff;
        if (h1 == 0 || h2 == 0){
            diff = h1 - h2;
        } else {
            diff = fmod(h1, h2);
            //if (diff < -180){
            //    diff += 360;
            // }
            //if (diff >= 180){
            //    diff -= 360;
            // }
        }
    
        
        // left
        if ( (diff < 135 && diff > 45) || (diff < -225 && diff > -315) ){
            return "10";
        }
        
        // right
        if ( (diff < 315 && diff > 225) || (diff < -45 && diff > -135) ){
            return "01";
        }
        
        return turnBit;
         */
    }
    
    std::string determineTurnBit(double prevRot, double currRot){
        // determines if a turn of greater than 20 degrees in either L/R has occurred
        // 00=no turn, 01=right turn, 10=left turn
        // to be added to BSD after rotation has been completed
        // compare PREVIOUS heading to NEW HEADING
        std::string turnBit = "00";
        prevRot = constrain(prevRot);
        currRot = constrain(currRot);
        double temp = currRot - prevRot;
        
        // temp = constrain(temp);
        if ( ((temp <= 180) && (temp > 45)) || ((temp < -180) && (temp > -315)) ){ // TODO correct?
            // left turn
            turnBit = "10";
        } else if ( ((temp > 180) && (temp < 315)) || (temp < 0 && temp > -180) ){
            // right turn
            turnBit = "01";
        }
        
        // TESTING
        std::cout<<"\ndetermineTurnBit-- preRot:"<<prevRot<<" currRot:"<<currRot<<" turnB: "<<turnBit;
        
        return turnBit;
    }
    
    int constrain (int heading) {
        while(heading < 0){
            heading+=360;
        }
        while (heading >= 360){
            heading-=360;
        }
        return heading;
    }
    
    bool hasRotation(double rot, double heading){
        // if true, switch is needed
        double temp = rot + heading;
        //temp = constrain(temp);
        if ( (temp > 90) && (temp < 270) ){
            return true;
        }
        return false;
    }
    
    /*
    std::string rotateTurn(std::string turnBit, double rot, double heading){
        
        std::cout<<"\nrotateTurn-- heading:"<<heading<<" rot:"<<rot;
        
        return turnBit;
        
         V2
        double diff = abs(rot - heading);
        //diff = constrain(diff);
        std::cout<<"\nrotateTurn-- diff: "<<diff;
        if ( diff < 225 && diff > 135){
            return switchTurn(turnBit);
        } return turnBit;
     

        //V1
        //if (hasRotation(rot, heading)){
        //    return switchTurn(turnBit);
        //}
        //return turnBit;
    }
    
    std::string switchTurn(std::string turnBit){
        std::string newTurnBit = "";
        // TESTING
        std::cout<<"---switchTURNBit---";
        if (turnBit.size() == 2){
            newTurnBit = turnBit[1];
            newTurnBit = newTurnBit + turnBit[0];
        } else {
            std::cout<<"\nPATH ERROR: turn bit is not expected length";
        }
        return newTurnBit;
    }*/
    
    std::string rotateWaypoint(Waypoint w, double heading){
        // if (nextWpHeading + heading is between (0 & 90) || (270 & 360) keep same BSD
        // else, switch (mirror) bsd (when between 91 * 269) switch <-- do this version
        double rot = w.rot;
        double h = heading;
        
        while (rot < -180){
            rot+=360;
        }
        while (rot > 180){
            rot-=360;
        }
        while (h < -180){
            h += 360;
        }
        while (h > 180){
            h -= 360;
        }
        
        // accepts input -180 to 180 range
        std::cout<<"\nrotateWaypoint()-- rot:"<<rot<<" h:"<<h;
        
        // skip if one value 0
        double diff;
        if (rot == 0 || h == 0){
            diff = abs(rot - h);
        } else {
            //diff = fmod(rot, h); WORKING UP TO 5s
            diff = abs(rot - h);
            if (diff < -360){
                diff += 360;
            }
            if (diff >= 360){
                diff -= 360;
            }
        }
        std::cout<<" diff: "<<diff;
        if (diff > (90+45) && diff < (180+45)){
            return switchBsd(w.bsd);
        }
        return w.bsd;
        
        /* V2
        // if heading is 180 degrees diff from rot, then switch bsd
        double diff = abs(w.rot - heading);
        std::cout<<"\nr:"<<w.rot<<" h:"<<heading;
        diff = constrain(diff);
        std::cout<<"\ndiff: "<<diff;
        if ( diff < 180 && diff >= 0){
            return w.bsd;
        } return switchBsd(w.bsd);
        */

        
        /* OLD METHOD V1
        double temp = w.rot + heading;
            temp = abs(temp);
            temp = constrain(temp);
        if ( (temp > 90) && (temp < 270) ){
            return switchBsd(w.bsd);
         } return w.bsd;
         */
    }
    
    std::string switchBsd(std::string bsd){
        // TESTING
        std::cout<<"---bsd rotating---";
        std::string newBsd = "";
        if (BSDLEN == 2){
            newBsd = bsd[1];
            newBsd = newBsd + bsd[0];
            return newBsd;
        } else if (BSDLEN == 6){ //TODO do this with bits, then convert to string it would be faster
            // assumes FB - L(d/w) - R(d/w)
            newBsd = bsd[1];
            newBsd = newBsd + bsd[0];
            newBsd = newBsd + bsd[4];
            newBsd = newBsd + bsd[5];
            newBsd = newBsd + bsd[2];
            newBsd = newBsd + bsd[3];
            return newBsd;
        } else if (BSDLEN == 4){
            // assumes FB LR
            newBsd = bsd[1];
            newBsd = newBsd + bsd[0];
            newBsd = newBsd + bsd[3];
            newBsd = newBsd + bsd[2];
            return newBsd;
        }
        std::cout<<"ERROR: CHECK switchBsd() for correct BSD length"<<std::endl;
        return bsd;
    }
    
    void corruptPath(std::bitset<(BIT_SIZE)> pathToCorrupt, int correctId){
        //std::cout<<"uncorrupted:"<<pathToCorrupt<<std::endl;
        cPath.setPath(pathToCorrupt, correctId);
        //cPath.printPath();
    }
    
    void distanceAllPaths(){
        // set all used values to 0
        reset();
        
        // loop over tree, and calculate distance between corruptPath and all other paths
        std::vector<SinglePath> allPaths = btree.getAllPaths();
        
        // pick 1 path randomly as target path, and corrupt it
        int index = rand() % allPaths.size();
        corruptPath(allPaths.at(index).path, allPaths.at(index).id);
        
        for(int i=0; i < allPaths.size(); i++){
            cPath.hamming(allPaths.at(i).path, allPaths.at(i).id);
        }
        
        // rank correct id and store information in another histogram
        cPath.setRank();
        int r = cPath.getRank();
        if (r > RANK_STATS_SIZE -1 ){
            r = RANK_STATS_SIZE -1;
        }
        rankHist[r] = rankHist[r] + 1;
    }
    
    void printStats(){
        cPath.printStats();
    }
    
    void printRankStats(){
        int firstRankTotal = rankHist[1];
        int nonFirstRankTotal = 0;
        float percentRank1;
        std::cout<<"RANK HISTOGRAM"<<std::endl;
        std::cout<<"ranks 1 to (Max-1) are true ranks. Ranks over the size of array are all added into final slot\n";
        std::cout<<"for ties, the correct id takes the lowest position\n\n";
        for(int i=1; i < RANK_STATS_SIZE; i++){
            std::cout<<std::setw(5)<<rankHist[i];
            // sum all ranks
            nonFirstRankTotal += rankHist[i];
        } std::cout<<std::endl;
        // print line
        for(int i=1; i < RANK_STATS_SIZE; i++){
            std::cout<<"-----";
        } std::cout<<std::endl;
        // print rank numbers
        for(int i=1; i < RANK_STATS_SIZE; i++){
            std::cout<<std::setw(5)<<i; //TODO should this be i+1?
        } std::cout<<std::endl;
        
        percentRank1 = ((float)firstRankTotal / (float)nonFirstRankTotal) * 100.0;
        std::cout<<"\n1st rank count: "<<firstRankTotal<<std::endl;
        std::cout<<"all other ranks: "<<nonFirstRankTotal<<std::endl;
        std::cout<<"\nFirst rank %: "<<percentRank1<<std::endl;
        std::cout<<std::endl;
    }
};

// ---------------------- //
// ------- MAIN --------- //
// ---------------------- //
int main(int argc, const char * argv[]) {
    
    srand((unsigned int)time(0));
    
    /*
     // find current working directy
     char * dir = getcwd(NULL, 0);
     std::cout << dir << std::endl;
     */
    // print info for file
    std::cout<<"Number of waypoints: "<<BRD_LEN<<std::endl;
    std::cout<<"Number of bits in BRD: "<<BRD_LEN * BSDLEN <<std::endl;
    std::cout<<"Number of bits with Turns: "<<BRD_LEN * BSDLEN + BRD_LEN <<std::endl;
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
    std::cout << "paths generated" << std::endl;
    
    // TESTING CORRUPTED PATHS FOR MATCHES
    std::cout << "corrupt path test:"<< std::endl;
    // pick a random unique path, and store final waypointId of path (this will be used for accuracy)
        // could randomly pick one while generating tree?
        // to start, I'll just pick a path
    
    // TODO LOOP OVER THIS X TIMES
    for(int i=0; i < 1; i++){
        p.distanceAllPaths();
        //p.printStats();
    }
    p.printRankStats();
    // TODO Print total rank stats
    
    return 0;
}
