/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <math.h>
#include <vector>
#include <string>
#include <map>

using namespace std;

enum STATE {SNT, WNT, WT, ST};

#define ADDRESS_SIZE 30
#define NOT_USING_SHARE 0
#define USING_SHARE_LSB 1
#define USING_SHARE_MID 2
#define VALID_BIT_SIZE 1
#define pass (void)0

//**************************************
// Global variables
//**************************************
class Btb;
class entry;
static Btb* btb = NULL;
SIM_stats simStats = SIM_stats();

// get the bitnr bit from n address
int getBit(uint32_t n, int bitnr) {
    int mask = 1 << bitnr;
    int masked_n = n & mask;
    int thebit = masked_n >> bitnr;
    return thebit;
}

// a class that allows control and insertion to an history register
class historyRegister{
    public:
        vector<bool> history;
        historyRegister(int historySize){
            this->history = vector<bool>(historySize,0);
        }
        historyRegister(const historyRegister& hr): history(hr.history){}
        void pushRight(bool bit){
                for (vector<bool>::iterator it = history.begin(); it != history.end(); it++){
                    if (it+1 != history.end())
                        *it = *(it+1);
                    else
                        *it = bit;
                }
            }
        string getHistory(){
            string res ="";
            for (vector<bool>::iterator it = history.begin(); it != history.end(); it++){
                char bit = *it?'1':'0';
                res.push_back(bit);
            }
            return res;
        }
        historyRegister& operator=(const historyRegister& hr){
            if (this == &hr)
                return *this;
            this->history = hr.history;
            return *this;
        }

};

// conversion from int to state enum
STATE int2State(int state){
    switch(state){
        case 0: return SNT;
        case 1: return WNT;
        case 2: return WT;
        case 3: return ST;
        default: return WNT;
    }
}

// branch target buffer class
// has all permutation for local/global history/table options
class Btb{
    public:
        unsigned btbSize;
        unsigned historySize;
        unsigned tagSize;
        STATE initialFsmState;
        bool isGlobalHist;
        bool isGlobalTable;
        bool Shared;
        historyRegister* globalHistory;
        map<string, STATE>* globalFsmTable;
        vector<entry*>* branchTable;
        Btb(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
            bool isGlobalHist, bool isGlobalTable, int Shared): btbSize(btbSize),historySize(historySize),
                                                                tagSize(tagSize),initialFsmState(int2State(fsmState)),
                                                                isGlobalHist(isGlobalHist),isGlobalTable(isGlobalTable),
                                                                Shared(Shared), globalHistory(NULL), globalFsmTable(NULL){
            this->branchTable = new  vector<entry*>(btbSize);
            for (int i =0; i < btbSize; i++){
                (*(this->branchTable))[i] = NULL;
            }
            if (isGlobalHist){
                this->globalHistory= new historyRegister(historySize);
            }
            if (isGlobalTable){
                this->globalFsmTable = new map<string, STATE>();
            }
        }
        ~Btb(){
            delete this->branchTable;
            delete this->globalHistory;
            delete this->globalFsmTable;
        }
        void addNewBranch(uint32_t pc);
        bool predict(uint32_t pc, uint32_t* dst);
        void update(uint32_t pc, uint32_t target_pc, bool taken, uint32_t pred_dst);
};
// provides the the key to the fsm entry, allows lsb/mid share
string getCurrentFsmEntry(historyRegister* history, uint32_t pc){
    if (!btb->isGlobalTable || btb->Shared == NOT_USING_SHARE){
        return history->getHistory();
    }
    uint32_t mask = 0;
    int startBit = 0;
    if (btb->Shared= USING_SHARE_LSB){
        startBit = 2;
    }
    else if (btb->Shared= USING_SHARE_MID){
        startBit = 16;
    }

    for (int i = 0; i < btb->historySize; i++){
        mask = mask << 1;
        mask++;
    }
    mask = mask << startBit;
    uint32_t pcWantedBits = mask & pc;
    string res = "";
    for (int i = 0; i< btb->historySize; i++){
        if (history->history[i] xor getBit(pcWantedBits,startBit+i)){
            res = "1"+res;
        }
        else {
            res = "0" + res;
        }
    }
    return res;
}

//takes the relevant bits from the address to the key in light of the btb size
int pc2key(uint32_t pc){
    int keySize = log2(btb->btbSize);
    uint32_t key_mask = 0;
    for (int i = 0; i < keySize; i++){
        uint32_t mask = 1 << i;
        key_mask = key_mask | mask;
    }
    key_mask = key_mask << 2;
    int key = (key_mask & pc);
    key = key >> 2;
    return key;
}

// takes the relevant bit for tag in light of the tag size
string calculateTag(uint32_t pc){
    string res = "";
    for (int i = 0; i < btb->tagSize; i++){
        int b = getBit(pc, i + btb->btbSize+2);
        char bit = b==1?'1':'0';
        res = bit + res;
    }
    return res;
}

//tag class, allows creating and retrieving a string representation
class Tag{
private:
    vector<bool> tag;
public:
    Tag(uint32_t pc,int tagSize){
        this->tag = vector<bool>(tagSize);
        for (int i = 0; i < tagSize; i++){
            tag[i] = getBit(pc, i+btb->btbSize+2);
        }
    }
    string getTag(){
        string res ="";
        for (vector<bool>::iterator it = tag.begin(); it != tag.end(); it++){
            char bit = *it?'1':'0';
            res = bit + res;
        }
        return res;
    }
};

// branch entry class, hold tag, local fms and local history when needed
class entry{
public :
    Tag* tag;
    historyRegister* history;
    map<string, STATE>* fsmTable;
    uint32_t predicted_pc;
    entry(uint32_t pc, int tagSize, int historySize): history(NULL), fsmTable(NULL), predicted_pc(pc+4) {

        this->tag = new Tag(pc, tagSize);
        if (!btb->isGlobalHist)
            this->history = new historyRegister(historySize);
        if (!btb->isGlobalTable)
            this->fsmTable = new map<string, STATE>();
    }
    ~entry(){
        delete tag;
        delete history;
        delete fsmTable;
    }
};

//add new branch to the btb
void Btb::addNewBranch(uint32_t pc){
    int key = pc2key(pc);
    delete (*branchTable)[key];
    (*branchTable)[key] = new entry(pc, btb->tagSize, btb->historySize);
}

//converts enum state to a true/false representation
bool state2Bool(STATE state){
    switch(state){
        case SNT: return false;
        case WNT: return false;
        case WT: return true;
        case ST: return true;
        default: return false;
    }
}

// predict taking/not taking the branch
bool Btb::predict(uint32_t pc, uint32_t* dst) {
    int key = pc2key(pc);
    map<string, STATE>* currentFsm;
    historyRegister* currentHistory;
    entry* currentEntry = (*(this->branchTable))[key];
    /*decide which table and history to use*/
    if (isGlobalTable){
        currentFsm = btb ->globalFsmTable;
    }
    else if (currentEntry != NULL) {
        currentFsm = (*(this->branchTable))[key]->fsmTable;
    }
    if (isGlobalHist){
        currentHistory = btb ->globalHistory;
    }
    else if (currentEntry != NULL) {
        currentHistory = (*(this->branchTable))[key]->history;
    }

    bool isTaken = false;
    if(currentEntry == NULL){ // no entry found
        isTaken = state2Bool(initialFsmState);
        *dst = pc+4;
    }
    else if (currentEntry->tag->getTag() != calculateTag(pc)) { // right entry wrong tag
         isTaken = state2Bool(initialFsmState);
         *dst = pc+4;
    }
    else { //right entry right tag
        if ((*currentFsm).find(getCurrentFsmEntry(currentHistory, pc)) == (*currentFsm).end()) { //no history entry in fsm
            isTaken = state2Bool(initialFsmState);
        }
        else { // found history entry in fsm
            isTaken = state2Bool(
                    (*currentFsm)[getCurrentFsmEntry(currentHistory, pc)]);
        }
        if (!isTaken) // return default target (pc+4)
            *dst = pc + 4;
        else { // return predicted target
            *dst = currentEntry->predicted_pc;
        }
    }
    return isTaken;
}
// taken given state and outcome returns next state
STATE updateState(STATE currentState, bool isTaken) {
    switch(currentState){
        case SNT:{
            if (isTaken) return WNT;
            return SNT;
        }
        case WNT:{
            if (isTaken) return WT;
            return SNT;
        }
        case WT:{
            if (isTaken) return ST;
            return WNT;
        }
        case ST:{
            if (isTaken) return ST;
            return WT;
        }
        default: return WNT;
    }
}

// update btb data
void Btb::update(uint32_t pc, uint32_t target_pc, bool taken, uint32_t pred_dst){
    simStats.br_num++;

    int key = pc2key(pc);
    map<string, STATE>* currentFsm;
    historyRegister* currentHistory;
    entry* currentEntry = (*(this->branchTable))[key];
    if (currentEntry == NULL){ // no entry found, insert new entry
        btb->addNewBranch(pc);
        currentEntry = (*(this->branchTable))[key];
    }
    uint32_t tempdst = 0;
    bool prediction =btb->predict(pc, &tempdst);
    if (currentEntry->tag->getTag() != calculateTag(pc)){ // wrong tag

        delete currentEntry;
        currentEntry = new entry(pc, btb->tagSize, btb->historySize);
    }
    /*decide which table and history to use*/
    if (isGlobalTable){
        currentFsm = btb ->globalFsmTable;
    }
    else {
        currentFsm = (*(this->branchTable))[key]->fsmTable;
    }
    if (isGlobalHist){
        currentHistory = btb ->globalHistory;
    }
    else {
        currentHistory = (*(this->branchTable))[key]->history;
    }
    if ((*currentFsm).find(getCurrentFsmEntry(currentHistory, pc)) == (*currentFsm).end()) {
        string newHistory = getCurrentFsmEntry(currentHistory, pc);
        (*currentFsm)[newHistory] = updateState(initialFsmState, taken);
    }
    else {
        (*currentFsm)[getCurrentFsmEntry(currentHistory, pc)] =
                updateState((*currentFsm)[getCurrentFsmEntry(currentHistory, pc)], taken);
    }
    if (taken == prediction){
        if (target_pc != pred_dst){
            currentEntry->predicted_pc = target_pc;
        }
    }
    else {
        simStats .flush_num++;
        currentEntry->predicted_pc = target_pc;
    }
    currentHistory->pushRight(taken);
    }
//calculate btb size
int calculateSize(){
    if (btb->isGlobalHist && btb->isGlobalTable){
        return btb->btbSize*(btb->tagSize + ADDRESS_SIZE + VALID_BIT_SIZE) + (btb->historySize + 2*pow(2,btb->historySize));
    }
    else if (btb->isGlobalHist){
        return btb->btbSize*(btb->tagSize + ADDRESS_SIZE + VALID_BIT_SIZE + 2*pow(2,btb->historySize)) + (btb->historySize);
    }
    else if (btb->isGlobalTable){
        return btb->btbSize*(btb->tagSize + ADDRESS_SIZE + VALID_BIT_SIZE + btb->historySize) + 2*(pow(2,btb->historySize));
    }
    else{
        return btb->btbSize*(btb->tagSize + ADDRESS_SIZE + VALID_BIT_SIZE + btb->historySize + 2*pow(2,btb->historySize));
    }
}
int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
    if (btbSize % 2 != 0 || btbSize < 1 || btbSize > 32)
        return -1;
    else if (historySize < 1 || historySize > 8)
        return -1;
    else if (tagSize < 0 || tagSize > 30 - log2(btbSize))
        return -1;
    else if (fsmState < 0 || fsmState > 3)
        return -1;
	btb = new Btb(btbSize, historySize, tagSize, fsmState, isGlobalHist, isGlobalTable, Shared);
	simStats.br_num =0;
	simStats.flush_num=0;
	simStats.size = calculateSize();
    return 0;

}

bool BP_predict(uint32_t pc, uint32_t *dst){
    return btb->predict(pc, dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
    btb->update(pc, targetPc, taken, pred_dst);
}

void BP_GetStats(SIM_stats *curStats){
    curStats->br_num = simStats.br_num;
    curStats->flush_num = simStats.flush_num;
    curStats->size = simStats.size;
    delete btb;
    return;
}

