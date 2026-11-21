// Control-flow unit test only. Scripted planner results do not validate paths or
// substitute for native planner/costmap/collision or Gazebo verification.
#include <behaviortree_cpp/bt_factory.h>
#include <nav2_behavior_tree/plugins/condition/are_error_codes_present_condition.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
struct Case {
  std::string name;
  unsigned primary, secondary;
  bool running_primary=false, running_secondary=false, halt_secondary=false;
};
struct State { Case test; std::vector<std::string> calls, halts; };
class Planner : public BT::StatefulActionNode {
 public:
  Planner(const std::string &name,const BT::NodeConfig &config,std::shared_ptr<State> state)
    :BT::StatefulActionNode(name,config),state_(state){}
  static BT::PortsList providedPorts(){return {
    BT::InputPort<std::string>("goal"),BT::InputPort<std::string>("planner_id"),
    BT::OutputPort<std::string>("path"),BT::OutputPort<uint16_t>("error_code_id")};}
  BT::NodeStatus onStart() override {
    if(!getInput("planner_id",planner_))throw std::runtime_error("planner_id unavailable");
    state_->calls.push_back(planner_);
    primary_=planner_=="GridBased";
    if(!primary_ && planner_!="GridBasedDijkstra")throw std::runtime_error("unexpected planner");
    if(primary_?state_->test.running_primary:state_->test.running_secondary)return BT::NodeStatus::RUNNING;
    return finish();
  }
  BT::NodeStatus onRunning() override{return finish();}
  void onHalted() override{state_->halts.push_back(planner_);}
 private:
  BT::NodeStatus finish(){
    auto code=static_cast<uint16_t>(primary_?state_->test.primary:state_->test.secondary);
    setOutput("error_code_id",code);setOutput("path",std::string(code?"":"unit-test path"));
    return code?BT::NodeStatus::FAILURE:BT::NodeStatus::SUCCESS;
  }
  std::shared_ptr<State>state_;std::string planner_;bool primary_;
};
void require(bool value,const std::string &message){if(!value)throw std::runtime_error(message);}
int main(int argc,char **argv){
 try{
  require(argc==2,"usage: verify_fallback EXTRACTED_TREE.xml");
  std::ifstream file(argv[1]);require(bool(file),"XML file unreadable");
  std::string xml((std::istreambuf_iterator<char>(file)),std::istreambuf_iterator<char>());
  std::vector<Case> cases={
    {"primary_success",0,208}, {"primary208_secondary_success",208,0},
    {"both_no_valid_path",208,208}, {"tf_error_no_retry",202,0},
    {"start_outside_no_retry",203,0}, {"goal_outside_no_retry",204,0},
    {"occupied_goal_no_retry",206,0}, {"secondary_failure_preserved",208,206},
    {"primary_running_before_fallback",208,0,true,false},
    {"secondary_running_not_starved",208,0,false,true},
    {"halt_running_secondary",208,0,false,true,true}};
  for(const auto &test:cases){
    auto state=std::make_shared<State>();state->test=test;
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<nav2_behavior_tree::AreErrorCodesPresent>("AreErrorCodesPresent");
    factory.registerBuilder<Planner>("ComputePathToPose",[state](const std::string &n,const BT::NodeConfig &c){return std::make_unique<Planner>(n,c,state);});
    auto board=BT::Blackboard::create();board->set("selected_planner",std::string("GridBased"));board->set("goal",std::string("unit-test goal"));
    auto tree=factory.createTreeFromText(xml,board);
    auto status=tree.tickExactlyOnce();
    if(test.running_primary){require(status==BT::NodeStatus::RUNNING,"primary must remain running");require(state->calls==std::vector<std::string>{"GridBased"},"secondary started before primary failure");status=tree.tickExactlyOnce();}
    if(test.running_secondary){
      require(status==BT::NodeStatus::RUNNING,"secondary must remain running");
      if(test.halt_secondary){tree.haltTree();require(state->halts==std::vector<std::string>{"GridBasedDijkstra"},"running secondary not halted");std::cout<<test.name<<" PASS\n";continue;}
      status=tree.tickExactlyOnce();
    }
    bool fallback=test.primary==208;
    auto expected=fallback?std::vector<std::string>{"GridBased","GridBasedDijkstra"}:std::vector<std::string>{"GridBased"};
    require(state->calls==expected,"incorrect planner dispatch order for "+test.name);
    auto code=fallback?test.secondary:test.primary;
    require(status==(code?BT::NodeStatus::FAILURE:BT::NodeStatus::SUCCESS),"incorrect final status for "+test.name);
    require(board->get<uint16_t>("compute_path_error_code")==code,"incorrect error code for "+test.name);
    std::cout<<test.name<<" PASS\n";
  }
  std::cout<<cases.size()<<" native BehaviorTree.CPP control-flow cases passed\n";return 0;
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
