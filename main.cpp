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
#define BRD_LEN 50 // number of waypoints in BRD
#define TOTAL_WAYPOINTS 284 // total number of waypoints entered in text file
#define MAXLINELEN 100 // maximum length of waypoint data for total waypoints < 1000 (000 00 heading x y 000 000 000....?)
#define BSDLEN 4 // number of bits in a BSD
#define DATAFILE "/Users/valiaodonnell/Documents/School/Bristol/masterProject/histogram/histogram/data/undirected/doors_walls_FBLR/distance2/data_distance2_284_FBLR.txt"
//test datafile
//#define DATAFILE "/Users/valiaodonnell/Documents/School/Bristol/masterProject/histogram/histogram/testData5.txt"
#define TESTDATAFILE "/Users/valiaodonnell/Documents/School/Bristol/masterProject/histogram/histogram/testData5.txt"
#define OUTPUT "/Users/valiaodonnell/Documents/School/Bristol/masterProject/histogram/histogram/histogram_output/output.txt"
// test mode
#define TEST_MODE_ACTIVE false
#define HEADING true // + 3 bits
#define TWO_BIT_TURN true // + 2 bits
// Change this depending on turn/heading bits (un)used
// 0 = no turn info
// 1 = 1 bit turn info
// 2 = 2 bit turn info
// heading is 3 bits: 0-N, 1-NE, 2-E, 3-SE, 4-S, 5-SW, 6-W, 7-NW where
// N=90, E=0, S=270, W=180
#define EXTRA_BITS 5 //number of bits added to semantic BSD from turns, heading etc

// distance stats
#define LOWER_RANGE 0
#define UPPER_RANGE 10
#define RANK_STATS_SIZE 11

// don't change
#define BSD_PLUS_EXTRA BSDLEN+EXTRA_BITS
#define BIT_SIZE BRD_LEN*(BSDLEN + EXTRA_BITS) // turn (2 bits - L/R)

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
        std::cout<<"\nPRINTING TREE: " << std::endl;
        std::string p = "";
        printTree(&root, p, BIT_SIZE);
        printStats();
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
        std::cout<<"Total paths : "<<sumStatsArray()<<std::endl;
        std::cout<<"Unique paths: "<<statsArray[1]<<std::endl;
        double percentUnique = ((float)statsArray[1] / (float)sumStatsArray()) * 100.0;
        std::cout<<"% Unique    : "<<percentUnique<<std::endl;
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
        //std::cout<<"\nCORRUPT: ";
        //std::cout<<"\npath:"<<corruptPath;
        // ASSUMES TURN BIT(s) OCCURS AS FIRST BIT OF BSD
        // so for a 2 bit BSD, it would be: turnbit=x, bsd=yy: xyyxyyxyyxyy
        // and it will only corrupt the y's and skip over the x's
        int extraBitCount = 0;
        // loop over bits and flip 25% of the time (for total accuracy of 75%)
        for (int i = (BIT_SIZE)-1; i >= 0; i--){ // not in reverse, but for binary, 0 starst on far right
            // flips bit 25% of the time, but leaves turn bits as is
            if (HEADING && !TWO_BIT_TURN){
                if ( (extraBitCount != 0) && (extraBitCount != 1) && (extraBitCount != 2) ){  // skips heading bits TODO - should these also be corrupted? accuracy of compass?
                    if (flip25()){
                        corruptPath[i] = !corruptPath[i];
                    }
                }
            } else if (HEADING && TWO_BIT_TURN){
                if ( (extraBitCount != 0) && (extraBitCount != 1) && (extraBitCount != 2) && (extraBitCount != 3) && extraBitCount != 4 ){  // skips heading bits && turnBits
                    if (flip25()){
                        corruptPath[i] = !corruptPath[i];
                    }
                }
            } else {
                if (flip25()){
                    corruptPath[i] = !corruptPath[i];
                }
            }
            extraBitCount++;
            extraBitCount = extraBitCount % (BSD_PLUS_EXTRA);
        }
        //std::cout<<"\ncoPa:"<<corruptPath;
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
        bool hasRotated = false;
        // access w.pos for current position
        // calculate heading for each new waypoint, and add w.bsd here in correct rotation
        double heading;
        if (path == ""){ // for 1st wp in path, heading is its own rotation
            heading = w.rot;
        } else {
            heading = calculateHeading(prevPos, w.pos); // (prevPos, nextPos)
        }
        
        // add turn bits, heading bits, and rotate BSD if needed:
        if (TWO_BIT_TURN){
            // add turn bit -- added BEFORE w BSD, since turn determined w/ prevPos to this wp
            // 2 bits (00=no turn, 10=left, 01=right, 11=not used)
            std::string turnBit;
            if (path.size() <= BSDLEN + 2){
                turnBit = "00"; // no turns for first 2 waypoints, as it will always be a straight line
            } else {
                //std::string turnBit = determineTurnBit(prevRot, w.rot);
                turnBit = determineTurnBit2(prevHeading, heading);
            }
            path = path + turnBit;
        }
        if (HEADING){
            // added after turn bit (if exists) and BEFORE bsd
            std::string headingBit = setHeadingBit(heading);
            path = path + headingBit;
        }
        if (DIRECTED){
            path = path + w.bsd;
        } else { // not directed
            //path = path + rotateTurn(turnBit, w.rot, heading) + rotateWaypoint(w, heading);
            // don't need to rotate turn bits, because calculation handles that already
            path = path + rotateWaypoint(w, heading, &hasRotated);
        }
        
        // add waypointId to vector of ids
        waypointIds.push_back(w.id);
        
        // check for connections if path length not reached yet
        if (path.length() < BIT_SIZE){ //
            // check every connection
            for(int i = 0; i < w.conns.size(); i++){
                // if connection has not been visited yet, visit it
                if (!(wa.waypointArray[w.conns[i]].visited)){
                    // need to pass in previous position to calculate heading
                    
                    wa.waypointArray[w.conns[i]].visited = true; // prevent loops
                    
                    // Retro-Active rotation for bsd's that couldn't be determined before
                    // (such as rotation of 1st waypoint, or ration at a junction)
                    if (!hasRotated){ // avoid double rotation
                        // check for 1st wp or TJunction here, and rotate bsd if needed ->
                        if ( (path.size() == BSD_PLUS_EXTRA) ){
                            // automatically rotates bsd for 1st bit
                            if ( !HEADING && !TWO_BIT_TURN ){
                                // delete last bsd from path (exluding turn info) and re-compute bsd orientation
                                path = path.substr(0, path.size()-BSDLEN);
                                double nextHeading = calculateHeading(w.pos, wa.waypointArray[w.conns[i]].pos);
                                path = path + retroRotate(w, nextHeading); // returns w's bsd
                            } else if ( HEADING && !TWO_BIT_TURN ){
                                // delete whole path and recompute for correct heading + bsd rotation
                                double nextHeading = calculateHeading(w.pos, wa.waypointArray[w.conns[i]].pos);
                                path = setHeadingBit(nextHeading);
                                path = path + retroRotate(w, nextHeading); // returns w's bsd
                            } else if ( HEADING && TWO_BIT_TURN){
                                double nextHeading = calculateHeading(w.pos, wa.waypointArray[w.conns[i]].pos);
                                path = determineTurnBit2(prevHeading, heading) + setHeadingBit(nextHeading);
                                path = path + retroRotate(w, nextHeading); // returns w's bsd
                            } else {
                                std::cout<<"ERROR: check retroRotation on build path";
                            }
                        } else if ( tJunction(w.rot, prevHeading) ){
                            // automatically rotates bsd if needed:
                            // delete last bsd from path (exluding turn info) and re-compute bsd orientation
                            path = path.substr(0, path.size()-BSDLEN);
                            double nextHeading = calculateHeading(w.pos, wa.waypointArray[w.conns[i]].pos);
                            path = path + retroRotate(w, nextHeading); // returns w's bsd
                        }
                    }
                    
                    // path, waypointIds, w, prevPos, prevRot, prevHeading, )
                    buildPath(path, waypointIds, wa.waypointArray[w.conns[i]], w.pos, w.rot, heading); // recursive call to continue building path
                    hasRotated = false;
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
            /*
            std::cout<<"\nPATH ADDED:";
            std::cout<<"\n"<<brd<<std::endl;
            for(int i=0; i < waypointIds.size(); i++){
                std::cout<<"   "<<waypointIds.at(i);
            } std::cout<<std::endl;
            */
            if (TEST_MODE_ACTIVE){
                testData5(path, waypointIds);
            }
        }
    }
    
    std::string setHeadingBit(double heading){
        std::string headingBit = "";
        // <= for upper bit, > for lower
        // for simplicity, cast heading as int, and shave off 0.5 on each segment, to
        // avoid floating point comparison
        int h = (int) constrain(heading);
        if (h <= 112 && h > 67){        // N - 000
            headingBit = "000";
        } else if (h <= 67 && h > 22){  // NE - 001
            headingBit = "001";
        } else if (h <= 22 || h > 337){ // E - 010
            headingBit = "010";
        } else if (h <= 337 && h > 292){// SE - 011
            headingBit = "011";
        } else if (h <= 292 && h > 247){// S - 100
            headingBit = "100";
        } else if (h <= 247 && h > 202){// SW - 101
            headingBit = "101";
        } else if (h <= 202 && h > 157){// W - 110
            headingBit = "110";
        } else if (h <= 157 && h > 112){// NW - 111
            headingBit = "111";
        }
        return headingBit;
    }
    
    std::string retroRotate(Waypoint w, double heading){
        // copy of rotateWaypoint without boolean values
        double diff = abs(constrain(w.rot) - heading);
        if ( (diff == 0) || (diff <= 90 && diff >= 0) || (diff >= 270 && diff <= 360) ){
            return w.bsd;
        }
        //std::cout<<"\n---RETRO ROTATE---";
        return switchBsd(w.bsd);
    }
    
    bool tJunction(double prevRot, double prevHeading){
        // prev heading intersected prev waypoint at a 90 degree angle, forming a T junction
        double diff = abs(prevRot - prevHeading);
        diff = constrain(diff);
        if ( (diff > 45 && diff < 135) || (diff > 225 && diff < 315) ){
            return true;
        }
        return false;
    }
    
    double long calculateHeading(Vector2d prevPos, Vector2d nextPos){
        Vector2d square = Vector2d();
        square.x = (nextPos.x - prevPos.x);
        square.y = (nextPos.y - prevPos.y);
        double long heading = atan2(square.y, square.x);
        
        if (heading >=0 ){
            heading = heading * 360/(2*M_PI);
        } else {
            heading = (2*M_PI + heading) * 360/(2*M_PI);
        }
        return heading;
    }
    
    std::string determineTurnBit2(double prevHeading, double heading){
        double h1 = prevHeading;
        double h2 = heading;
        
        // constrain
        h1 = constrain(h1);
        h2 = constrain(h2);
        double diff = constrain(h2 - h1);
        
        if (diff > 225 && diff < 315){
            // right turn
            return "01";
        } else if (diff > 45 && diff < 135){
            // left turn
            return "10";
        }
        return "00";
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
    
    std::string rotateWaypoint(Waypoint w, double heading, bool* hasRotated){
        double diff = abs(constrain(w.rot) - heading);
        if ( (diff == 0) || (diff <= 90 && diff >= 0) || (diff >= 270 && diff <= 360) ){
            *hasRotated = false;
            return w.bsd;
        }
        *hasRotated = true;
        return switchBsd(w.bsd);
    }
    
    std::string switchBsd(std::string bsd){
        // has no effect on turn bits
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
        if (!allPaths.empty()){
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
    }
    
    void printUniqueStats(){
        btree.print();
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
    
    // ---------------------- //
    // ---- PATH TESTING ---- //
    // ---------------------- //
    
    void test(){
        std::cout<<"\nPath tests initiated";
        testConstrain();
        testHeading();
        testRotateWaypoint();
        testDetermineTurnBit2();
        testTJunction();
        // test retroActiveRotation
        std::cout<<"\nPath tests complete";
    }
    
    void testConstrain(){
        std::cout<<"\nTesting Constrain(): ";
        assert(constrain(90)    == 90);
        assert(constrain(360)   == 0);
        assert(constrain(-135)  == 225);
        assert(constrain(-45)   == 315);
        
        std::cout<<" -- constrain tests passed";
    }
    
    void testHeading(){
        std::cout<<"\nTesting Heading():";
        // test1
        Vector2d a = Vector2d(); a.x = 3; a.y = 1;
        Vector2d b = Vector2d(); b.x = 3; b.y = 3;
        Vector2d c = Vector2d(); c.x = 1; c.y = 3;
        Vector2d d = Vector2d(); d.x = 1; d.y = 1;
        Vector2d e = Vector2d(); e.x = 3; e.y = -3;
        Vector2d f = Vector2d(); f.x = 2; f.y = 2;
        
        double h1 = calculateHeading(a, b);
        double h2 = calculateHeading(b, a);
        double h3 = calculateHeading(b, c);
        double h4 = calculateHeading(c, b);
        double h5 = calculateHeading(a, e);
        double h6 = calculateHeading(d, f);
        double h7 = calculateHeading(c, f);
        double h8 = calculateHeading(b, f);
        double h9 = calculateHeading(a, f);
        
        assert((int) h1 == 90);
        assert((int) h2 == 270);
        assert((int) h3 == 180);
        assert((int) h4 == 0);
        assert((int) h5 == 270);
        assert((int) h6 == 45);
        assert((int) h7 == 315);
        assert((int) h8 == 225);
        assert((int) h9 == 135);
        std::cout<<" --Heading tests passed";
    }
    
    void testRotateWaypoint(){
        // construct waypoints
        std::cout<<"\nTesting RotateWaypoint(): ";
        Waypoint w1 = Waypoint();
        w1.rot = 0;
        Vector2d pos1 = Vector2d(); pos1.x = 3; pos1.y = 1;
        w1.pos = pos1;
        
        // test path in all directions
        bool hasRotated = false;
        w1.bsd = "01";
        std::string p1 = rotateWaypoint(w1, 0, &hasRotated);
        assert(p1 == "01");
        assert(hasRotated == false);
        p1 = rotateWaypoint(w1, 180, &hasRotated);
        assert(p1 == "10");
        assert(hasRotated == true);
        hasRotated = false;
        w1.bsd = "01";
        p1 = rotateWaypoint(w1, 90, &hasRotated);
        assert(p1 == "01");
        hasRotated = false;
        // test edge cases
        p1 = rotateWaypoint(w1, 91, &hasRotated);
        assert(p1 == "10");
        hasRotated = true;
        w1.bsd = "01";
        p1 = rotateWaypoint(w1, 269, &hasRotated);
        assert(p1 == "10");
        hasRotated = true;
        
        Waypoint w2 = Waypoint();
        w2.rot = -45;
        Vector2d pos2 = Vector2d(); pos2.x = 3; pos2.y = 1;
        w2.pos = pos2;
        w2.bsd = "01";
        hasRotated = false;
        std::string p2 = rotateWaypoint(w2, 0, &hasRotated);
        assert(p2 == "01");
        assert( hasRotated == false);
        p2 = rotateWaypoint(w2, 99, &hasRotated);
        assert(p2 == "10");
        w2.bsd = "01";
        p2 = rotateWaypoint(w2, 45, &hasRotated);
        assert(p2 == "01");
        
        Waypoint w3 = Waypoint();
        w3.rot = -225;
        Vector2d pos3 = Vector2d(); pos3.x = 3; pos3.y = 1;
        w3.pos = pos3;
        w3.bsd = "01";
        std::string p3 = rotateWaypoint(w3, 0, &hasRotated);
        assert(p3 == "10");
        w3.bsd = "01";
        p3 = rotateWaypoint(w3, 90, &hasRotated);
        assert(p3 == "01");
        w3.bsd = "01";
        p3 = rotateWaypoint(w3, 45, &hasRotated);
        assert(p3 == "01");
        
        std::cout<<" -- rotateWaypoint tests passed";
    }
    
    void testDetermineTurnBit2(){
        std::cout<<"\nTesting DetermineTurnBit(): ";
        //  std::string determineTurnBit(double prevRot, double currRot)
        assert( "00" == determineTurnBit2(0, 0) );
        assert( "00" == determineTurnBit2(180, 180) );
        assert( "00" == determineTurnBit2(0, 5) );
        assert( "00" == determineTurnBit2(0, 40) );
        assert( "00" == determineTurnBit2(90, 46) );
        assert( "00" == determineTurnBit2(-135, 230) );
        assert( "00" == determineTurnBit2(135, 180) );

        assert( "01" == determineTurnBit2(0, 314) );
        assert( "01" == determineTurnBit2(0, 226) );
        assert( "01" == determineTurnBit2(0, -46) );
        assert( "01" == determineTurnBit2(180, 90) );
        assert( "01" == determineTurnBit2(180, 47) );

        assert( "10" == determineTurnBit2(0, 90) );
        assert( "10" == determineTurnBit2(-135, -46) );
        assert( "10" == determineTurnBit2(180, 270) );
        assert( "10" == determineTurnBit2(225, 300) );
        assert( "10" == determineTurnBit2(0, 99) );
        assert( "10" == determineTurnBit2(360, 90) );
        
        std::cout<<" -- turnBit tests passed";
    }
    
    void testTJunction(){
        std::cout<<"\nTesting TJunction(): ";
        assert( tJunction( 0, 90)   == true );
        assert( tJunction( 180, 95) == true );
        assert( tJunction( 180, 46) == true );
        
        assert( tJunction( 0, 45)   == false );
        assert( tJunction( 0, 10)   == false );
        assert( tJunction( 0, -45)   == false );
        std::cout<<" -- TJunction tests passed";
    }
    
    void testData5(std::string path, std::vector<int> waypointIds){
        // convert waypoint ids to a string
        std::string ids = "";
        for(int i=0; i < waypointIds.size(); i++){
            ids = ids + std::to_string(waypointIds.at(i));
        }
        
        if (HEADING && !TWO_BIT_TURN){
            // 1s
            if ( ids == "1234" ){
                assert( path == "01000010010101010011");
            } else if ( ids == "1275" ){
                assert( path == "01000010011001110001");
            } else if ( ids == "1657"){
                assert( path == "10000100000101000011"); // CHANGED FOR RETRO ROTATE
            } else if ( ids == "1654"){
                assert( path == "10000100000101001011"); // changed for RR
            }
            // 2s
            else if( ids == "2345" ){
                assert( path == "01001010101001111001" );
            } else if ( ids == "2754" ){
                assert( path == "10001100111001001011" ); // retroRotate
            } else if ( ids == "2756" ){
                assert( path == "10001100111000111000" );
            } else if ( ids == "2165" ){ // RR
                assert( path == "11010110001000001010" );
            }
            // 3s
            else if( ids == "3457" ){
                assert( path == "10010100111100100011" );
            } else if ( ids == "3456" ){
                assert( path == "10010100111100111000");
            } else if ( ids == "3216" ){
                assert( path == "11010110101100010000");
            } else if ( ids == "3275" ){
                assert( path == "11010110101001110001");
            }
            // 4s
            else if( ids == "4572" ){
                assert( path == "11011110010001100001" );
            } else if ( ids == "4561" ){
                assert( path == "11011110011100000000");
            } else if ( ids == "4327" ){
                assert( path == "00011000011101010011");
            } else if ( ids == "4321" ){
                assert( path == "00011000011101011000");
            }
            // 5s
            else if( ids == "5612" ){
                assert( path == "11001110000000001001" );
            } else if ( ids == "5721" ){
                assert( path == "00001000110001011000"); // RETRO_ACTIVE TJUNCTION
            } else if ( ids == "5723" ){
                assert( path == "00001000110000101010");
            } else if ( ids == "5432" ){ // retroActive
                assert( path == "01010010110000111010");
            }
            // 6s
            else if ( ids == "6572" ){
                assert( path == "01000010100001100001" );
            } else if ( ids == "6543" ){
                assert( path == "01000010100101100001");
            } else if ( ids == "6127" ){
                assert( path == "00000000000100110011");
            } else if ( ids == "6123" ){
                assert( path == "00000000000100101010");
            }
            // 7s
            else if ( ids == "7561" ){
                assert( path == "10011100011100000000" ); // RR
            } else if ( ids == "7543" ){
                assert( path == "10011100100101100001" ); // RR
            } else if ( ids == "7216" ){
                assert( path == "00011000101100010000"); // RR
            } else if ( ids == "7234" ){
                assert( path == "00011000010101010011");
            }

        } else if (TWO_BIT_TURN && !HEADING){
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
                assert( path == "0001001100101011" ); // retroRotate
            } else if ( ids == "2756" ){
                assert( path == "0001001100010100" );
            } else if ( ids == "2165" ){
                assert( path == "0010000010001010" );
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
                assert( path == "0001001100101000"); // RETRO_ACTIVE TJUNCTION
            } else if ( ids == "5723" ){
                assert( path == "0001001100010110");
            } else if ( ids == "5432" ){
                assert( path == "0010001110011010");
            }
            // 6s
            else if ( ids == "6572" ){
                assert( path == "0000001010110001" );
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
                assert( path == "0011001010111001" ); // rotate 2 if adding retr-active rotation
            } else if ( ids == "7216" ){
                assert( path == "0011001010001000"); // rotate 2 if adding retr-active rotation
            } else if ( ids == "7234" ){
                assert( path == "0011000101100111");
            }
        }
    }
};

// ---------------------- //
// ------- MAIN --------- //
// ---------------------- //
int main(int argc, const char * argv[]) {
    
    srand((unsigned int)time(0));
    
    if (TEST_MODE_ACTIVE){
        std::cout<<"\nTEST MODE ACTIVE"<<std::endl;
        
        // the following tests are ONLY for testData5
        std::string data;
        WaypointArray wa = WaypointArray(); // single waypoint array used throughout
        std::map<std::string, int> pathMap; // single map used throughout
        
        // generate string from file, builds waypoints from lines, stores them to waypoint array
        DataProcessing dp = DataProcessing(TESTDATAFILE);
        dp.process(&data, &wa);
        std::cout << "data processed to waypoint array" << std::endl;
        
        // generate paths
        Paths p = Paths(BRD_LEN, &wa); // takes into account waypoint rotation and direction of travel
        p.test();
        p.generatePaths();
        
        //p.generatePaths();
        std::cout << "\nTests completed successfully" << std::endl;
        
    } else {
        /*
         // find current working directy
         char * dir = getcwd(NULL, 0);
         std::cout << dir << std::endl;
         */
        // print info for file
        std::cout<<"Number of waypoints: "<<BRD_LEN<<std::endl;
        //std::cout<<"Number of bits in BRD: "<<BRD_LEN * BSDLEN <<std::endl;
        std::cout<<"Number of bits: "<<BRD_LEN * (BSDLEN + EXTRA_BITS) <<std::endl;
        std::cout<<"Total number of waypoints: "<<TOTAL_WAYPOINTS<<std::endl;
        std::cout<<"data file: "<<DATAFILE<<std::endl<<std::endl;;
        
        // pick path length
        std::string data;
        WaypointArray wa = WaypointArray(); // single waypoint array used throughout
        std::string fileName = DATAFILE;
        std::map<std::string, int> pathMap; // single map used throughout
        
        // generate string from file, builds waypoints from lines, stores them to waypoint array
        DataProcessing dp = DataProcessing(fileName);
        dp.process(&data, &wa);
        std::cout << "data processed to waypoint array" << std::endl;
        
        // generate paths
        Paths p = Paths(BRD_LEN, &wa); // takes into account waypoint rotation and direction of travel
        p.generatePaths();
        std::cout << "paths generated" << std::endl;
        
        // number of unique paths:
        //p.printUniqueStats();
        
        
        // TESTING CORRUPTED PATHS FOR MATCHES
        std::cout << "corrupt path testing"<< std::endl;
        // pick a random unique path, and store final waypointId of path (this will be used for accuracy)
        // could randomly pick one while generating tree?
        // to start, I'll just pick a path
        
        // TODO LOOP OVER THIS X TIMES
        for(int i=0; i < 1000; i++){
            p.distanceAllPaths();
            //p.printStats();
        }
        p.printRankStats();
        // TODO Print total rank stats
        
    }
    return 0;
}
