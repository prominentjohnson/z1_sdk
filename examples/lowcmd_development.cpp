#include "unitree_arm_sdk/control/unitreeArm.h"
#include <fstream>
#include <chrono>
#include <algorithm>

using namespace UNITREE_ARM;

Vec6 clampVec6(const Vec6& v, double low, double high)
{
    Vec6 result;
    for (int i = 0; i < 6; i++) {
        result[i] = std::max(low, std::min(v[i], high));
    }
    return result;
}

int main(int argc, char *argv[]) {
    using clock = std::chrono::steady_clock;
    std::ifstream file("../examples/nominal_trajectory.csv");
    if (!file.is_open()) {
        std::cerr << "Error opening file\n";
        return 1;
    }

    std::string line;
    std::getline(file, line); // skip header

    std::vector<std::vector<double>> rows;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> row;
        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stod(cell));
        }
        rows.push_back(row);
    }

    // Convert to Eigen matrix
    size_t nrows = rows.size();
    size_t ncols = rows[0].size();
    Eigen::MatrixXd data(nrows, ncols);
    for (size_t i = 0; i < nrows; ++i)
        for (size_t j = 0; j < ncols; ++j)
            data(i, j) = rows[i][j];

    // Extract back into arrays
    Eigen::VectorXd t_interp = data.col(0);
    Eigen::MatrixXd q_interp = data.block(0, 1, nrows, 6).transpose();
    Eigen::MatrixXd q_dot_interp = data.block(0, 7, nrows, 6).transpose();
    Eigen::MatrixXd tau_interp = data.block(0, 13, nrows, 6).transpose();

    // std::cout << "Loaded " << nrows << " rows.\n";
    // std::cout << "First time: " << t(0) << "\n";
    // std::cout << "First q: " << q.col(0).transpose() << "\n";
    double dt = t_interp(1) - t_interp(0);
    Vec6 q_init = q_interp.col(0);
    Vec6 q_end = q_interp.col(q_interp.cols() - 1);

    std::cout << std::fixed << std::setprecision(3);
    bool hasGripper = true;
    unitreeArm arm(hasGripper);
    arm.sendRecvThread->start();

    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.setFsm(ArmFSMState::LOWCMD);

    // std::vector<double> KP, KW;
    // KP = arm._ctrlComp->lowcmd->kp;
    // KW = arm._ctrlComp->lowcmd->kd;

    std::vector<double> KP = {20, 30, 30, 20, 15, 10};
    std::vector<double> KW = {2000, 2000, 2000, 2000, 2000, 2000};
    arm._ctrlComp->lowcmd->setControlGain(KP, KW);    
    Eigen::Map<const Vec6> KP_Eigen(KP.data());
    Eigen::Map<const Vec6> KW_Eigen(KW.data());
    // arm._ctrlComp->lowcmd->setGripperGain(KP[KP.size()-1], KW[KW.size()-1]);
    arm.sendRecvThread->shutdown();

    Vec6 initQ = arm.lowstate->getQ();

    double duration = 300;
    Vec6 targetQ;
    targetQ << q_init;
    Vec6 outputTau;
    Timer timer(dt);
    for(int i(0); i<duration; i++){
        arm.q = initQ * (1-i/duration) + targetQ * (i/duration);
        arm.qd = (targetQ - initQ) / (duration * arm._ctrlComp->dt);
        outputTau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
        arm.tau = clampVec6(outputTau, -30.0, 30.0);
        arm.gripperQ = 0;
        std::cout << arm.tau.transpose() << "\n";
        
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
        arm.sendRecv();
        timer.sleep();
    }

    double warmup_time = 2.0;  // seconds
    auto start_time = clock::now();
    while (std::chrono::duration<double>(clock::now() - start_time).count() < warmup_time) {
        arm.q << q_init;
        arm.qd << 0,0,0,0,0,0;
        outputTau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
        arm.tau = clampVec6(outputTau, -30.0, 30.0);
        arm.gripperQ = 0;
        std::cout << arm.tau.transpose()<<"\n";
        
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
        arm.sendRecv();
        timer.sleep();
    }

    int catching_steps = t_interp.size();
    Vec6 currentQ;
    Vec6 currentQd;
    Vec6 desiredQ;
    Vec6 desiredQd;
    Vec6 feedforwardTau;
    Vec6 currentTau;
    std::vector<double> kp = {750, 750, 750, 750, 750, 750};
    std::vector<double> kd = {20, 20, 20, 20, 20, 20};
    Eigen::Map<const Vec6> kp_Eigen(kp.data());
    Eigen::Map<const Vec6> kd_Eigen(kd.data());
    KP = {0, 0, 0, 0, 0, 0};
    KW = {0, 0, 0, 0, 0, 0};
    arm._ctrlComp->lowcmd->setControlGain(KP, KW);
    for(int i(0); i<catching_steps; i++){
        desiredQ << q_interp.col(i);
        desiredQd << 0,0,0,0,0,0;//q_dot_interp.col(i);
        feedforwardTau << 0,0,0,0,0,0;//tau_interp.col(i);
        currentQ = arm.lowstate->getQ();
        currentQd = arm.lowstate->getQd();
        outputTau = kp_Eigen.array() * (desiredQ - currentQ).array() + kd_Eigen.array()* (desiredQd - currentQd).array() + feedforwardTau.array();
        arm.tau = clampVec6(outputTau, -30.0, 30.0);
        arm.gripperQ = 0;

        currentTau = arm.lowstate->getTau();
        std::cout << arm.tau.transpose()<<"\n";
        // std::cout << currentTau.transpose() << "and\t"<< arm.tau.transpose()<< "\n";
        // std::cout << currentTau.transpose() << "taucmd:\n"<< arm.tau.transpose()<< "\n";
        std::cout << desiredQ.transpose()<<"\n" << currentQ.transpose() << "difference:"<< (desiredQ-currentQ).transpose() <<"\n";
        
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
        arm.sendRecv();
        timer.sleep();
    }

    // double duration = 1000;
    // Vec6 targetQ;
    // Vec6 currentQ;
    // Vec6 currentQd;
    // Vec6 currentTau;
    // Eigen::Map<const Vec6> KP_Eigen(KP.data());
    // Eigen::Map<const Vec6> KW_Eigen(KW.data());
    // Vec6 outputTau;
    // targetQ << q_init;
    // Timer timer(arm._ctrlComp->dt);
    // for(int i(0); i<duration; i++){
    //     arm.q = initQ * (1-i/duration) + targetQ * (i/duration);
    //     arm.qd = (targetQ - initQ) / (duration * arm._ctrlComp->dt);
    //     arm.tau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
    //     arm.gripperQ = 0;
    //     currentQ = arm.lowstate->getQ();
    //     currentQd = arm.lowstate->getQd();
    //     outputTau = 25.6 * KP_Eigen.array() * (arm.q - currentQ).array() + 0.0128 * KW_Eigen.array()* (arm.qd - currentQd).array() + arm.tau.array();
    //     currentTau = arm.lowstate->getTau();
    //     // std::cout << outputTau << "and" << currentTau << "end";
        
    //     arm.setArmCmd(arm.q, arm.qd, arm.tau);
    //     arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
    //     arm.sendRecv();
    //     timer.sleep();
    // }

    KP = {20, 30, 30, 20, 15, 10};
    KW = {2000, 2000, 2000, 2000, 2000, 2000};
    arm._ctrlComp->lowcmd->setControlGain(KP, KW);
    double holding_time = 2.0;  // seconds
    auto finish_time = clock::now();
    while (std::chrono::duration<double>(clock::now() - finish_time).count() < holding_time) {
        arm.q << q_end;
        arm.qd << 0,0,0,0,0,0;
        outputTau = arm._ctrlComp->armModel->inverseDynamics(arm.q, arm.qd, Vec6::Zero(), Vec6::Zero());
        arm.tau = clampVec6(outputTau, -30.0, 30.0);
        arm.gripperQ = 0;
        std::cout << arm.tau.transpose()<<"\n";
        
        arm.setArmCmd(arm.q, arm.qd, arm.tau);
        arm.setGripperCmd(arm.gripperQ, arm.gripperW, arm.gripperTau);
        arm.sendRecv();
        timer.sleep();
    }

    arm.sendRecvThread->start();

    arm.setFsm(ArmFSMState::JOINTCTRL);
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    return 0;
}