/* 046267 Computer Architecture - Winter 20/21 - HW #1                  */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <math.h>
#include <vector>
#include <string>
#include <map>

using namespace std;

enum STATE {SNT, WNT, WT, ST};

//**************************************
// Global variables
//**************************************
class Btb;
class entry;
static Btb* btb = NULL;


int getBit(uint32_t n, int bitnr) {
    int mask = 1 << bitnr;
    int masked_n = n & mask;
    int thebit = masked_n >> bitnr;
    return thebit;
}

class Tag{
    private:
        vector<bool> tag;
    public:
        Tag(uint32_t pc,int tagSize){
            this->tag = vector<bool>(tagSize);
            for (int i = 0; i < tagSize; i++){
                tag[i] = getBit(pc, i);
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



class historyRegister{
    private:
        vector<bool> history;
    public:
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
STATE int2State(int state){
    switch(state){
        case 0: return SNT;
        case 1: return WNT;
        case 2: return WT;
        case 3: return ST;
        default: return WNT;
    }
}

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
            this->branchTable = new  vector<entry*>(log2(btbSize));
            for (int i =0; i < log2(btbSize); i++){
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
//        void update(uint32_t pc, uint32_t target_pc, bool taken, uint32_t pred_dst);

};

int pc2key(uint32_t pc){
    int keySize = log2(btb->btbSize);
    uint32_t key = 0;
    for (int i = 0; i < keySize; i++){
        uint32_t mask = 1 << i;
        key = key | mask;
    }
    return (key & pc);
}

string calculateTag(uint32_t pc){
    string res = "";
    for (int i = 0; i < btb->tagSize; i++){
        int b = getBit(pc, i);
        char bit = b==1?'1':'0';
        res = bit + res;
    }
    return res;
}

class entry{
public :
    Tag* tag;
    historyRegister* history;
    map<string, STATE>* fsmTable;
    uint32_t predicted_pc;
    bool valid;
    entry(uint32_t pc, int tagSize, int historySize): history(NULL), fsmTable(NULL), predicted_pc(pc+4), valid(false) {

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
void Btb::addNewBranch(uint32_t pc){
    int key = pc2key(pc);
    delete (*branchTable)[key];
    (*branchTable)[key] = new entry(pc, btb->tagSize, btb->historySize);
}

bool state2Bool(STATE state){
    switch(state){
        case SNT: return false;
        case WNT: return false;
        case WT: return true;
        case ST: return true;
        default: return false;
    }
}

bool Btb::predict(uint32_t pc, uint32_t* dst) {
    int key = pc2key(pc);
    map<string, STATE>* currentFsm;
    historyRegister* currentHistory;
    entry* currentEntry = (*(this->branchTable))[key];

    if (currentEntry == NULL){
        btb->addNewBranch(pc);
    }

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

    bool isTaken = false;
    if (currentEntry->tag->getTag() != calculateTag(pc)) {
         isTaken = state2Bool(initialFsmState);
         *dst = pc+4;
    }
    else {
        if ((*currentFsm).find(currentHistory->getHistory()) == (*currentFsm).end()) {
            isTaken = state2Bool(initialFsmState);
        }
        else {
            isTaken = state2Bool(
                    (*currentFsm)[currentEntry->history->getHistory()]); // todo: change to func for lshare/gshare
        }
        if (!isTaken)
            *dst = pc + 4;
        else {
            *dst = currentEntry->predicted_pc;
        }
    }
    return isTaken;
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
    return 0;

}

bool BP_predict(uint32_t pc, uint32_t *dst){
    return btb->predict(pc, dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	return;
}

void BP_GetStats(SIM_stats *curStats){
	return;
}

