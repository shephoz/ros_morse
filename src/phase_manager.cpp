#include <vector>
using namespace std;

#include "vec.cpp"


class PhaseManager{
    public:
        PhaseManager();
        ~PhaseManager();
        void addPhase(double x, double y, double yaw);
        Vec getGoal();
        void nextPhase();
        bool isEnd();
        int num();
        bool isAttituding();
        void endAttituding();
    private:
        int phase_number_;
        bool attituding_;
        vector< vector<double> > goals_;
};

PhaseManager::PhaseManager(){
    phase_number_ = 0;
    attituding_ = true;
}

PhaseManager::~PhaseManager(){

}

void PhaseManager::addPhase(double x, double y, double yaw){
    Vec<double> goal(x,y,yaw);
    goals_.push_back(goal);
}

Vec PhaseManager::getGoal(){
    //return goals_[phase_number_]
    return null;
}

void PhaseManager::nextPhase(){
    //goals_.erase(goals_.begin());
    phase_number_++;
    attituding_ = true;
}

bool PhaseManager::isEnd(){
    return phase_number_ >= goals_.size();
}

int PhaseManager::num(){
    return phase_number_;
}


bool PhaseManager::isAttituding(){
    return attituding_;
}

void PhaseManager::endAttituding(){
    attituding_ = false;
}
