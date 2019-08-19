#include "ortools/linear_solver/linear_solver.h"
using namespace std;
namespace operations_research {

pair<vector<vector<int>>,pair<vector<vector<bool>>,vector<vector<bool>>>> simple_mip_program(
        int day, 
        int basic, 
        int goal,
        vector<int> time_interval, 
        vector<int> time_cost, 
        vector<pair<int,int>> gift_coef, 
        vector<int> stone_for_gift) {
  // Create the mip solver with the CBC backend.
  MPSolver solver("simple_mip_program",
                  MPSolver::CBC_MIXED_INTEGER_PROGRAMMING);

  const double infinity = solver.infinity();
  // x and y are integer non-negative variables.
  vector<vector<MPVariable*>> num_of_times;
  vector<vector<MPVariable*>> activate;
  vector<vector<MPVariable*>> buy_or_not;
  for(int i=0;i<day;i++)
  {
    vector<MPVariable*> temp0 = {solver.MakeIntVar(0,time_interval[0],to_string(i)+"0")};
    for(int j=0;j<time_interval.size()-1;j++)
    {
      temp0.push_back(solver.MakeIntVar(0,time_interval[j+1]-time_interval[j],to_string(i)+to_string(j+1)));
    }
    num_of_times.push_back(temp0);
    vector<MPVariable*> temp1;
    vector<MPVariable*> temp2;
    for(int j=0;j<time_interval.size();j++)
    {
      temp1.push_back(solver.MakeBoolVar("a("+to_string(i)+","+to_string(j)+")"));
      temp2.push_back(solver.MakeBoolVar("b("+to_string(i)+","+to_string(j)+")"));
    }
    activate.push_back(temp1);
    buy_or_not.push_back(temp2);
  }

  LOG(INFO) << "Number of variables = " << solver.NumVariables();
  vector<MPConstraint*> ActivateConstraint;
  for(int i=0;i<day;i++)
  {
    // for(int j=0;j<4;j++)
    // {
    //   MPConstraint* temp = solver.MakeRowConstraint(0,infinity,"act day"+to_string(i)+": "+to_string(j)+","+to_string(i+1));
    //   temp->SetCoefficient(activate[i][j],1);
    //   temp->SetCoefficient(activate[i][j+1],-1);
    //   ActivateConstraint.push_back(temp);
    // }
    for(int j=0;j<time_interval.size()-1;j++)
    {
      MPConstraint* temp = solver.MakeRowConstraint(0,infinity,"");
      temp->SetCoefficient(activate[i][j],time_interval[j+1]-time_interval[j]);
      temp->SetCoefficient(num_of_times[i][j+1],-1);
      ActivateConstraint.push_back(temp);
    }
    for(int j=0;j<time_interval.size();j++)
    {
      MPConstraint* temp = solver.MakeRowConstraint(0,infinity,"act day"+to_string(i)+": "+to_string(j));
      for(int k=0;k<=j;k++){
        temp->SetCoefficient(num_of_times[i][k],1);
      }
      temp->SetCoefficient(activate[i][j],-time_interval[j]);
      ActivateConstraint.push_back(temp);
    }

    //Must activate before buy
    for(int j=0;j<time_interval.size();j++)
    {
      MPConstraint* temp = solver.MakeRowConstraint(0,infinity,""); 
      temp->SetCoefficient(activate[i][j],1);
      temp->SetCoefficient(buy_or_not[i][j],-1);
      ActivateConstraint.push_back(temp);
    }

    MPConstraint* temp = solver.MakeRowConstraint(1-time_interval.back(),infinity,"");
    temp->SetCoefficient(activate[i].back(),1);
    for(int j=0;j<time_interval.size();j++)
    {
      temp->SetCoefficient(num_of_times[i][j],-1);
    }
    ActivateConstraint.push_back(temp);
    
  }

  //NEED TO MODIFY
  MPConstraint* gift_require = solver.MakeRowConstraint(goal,infinity,"Requirement");
  for(int i=0;i<day;i++)
  {
    for(int j=0;j<time_interval.size();j++)
    {
      gift_require->SetCoefficient(num_of_times[i][j],basic);
      gift_require->SetCoefficient(activate[i][j],gift_coef[j].first);
      gift_require->SetCoefficient(buy_or_not[i][j],gift_coef[j].second-gift_coef[j].first);
    }
  }
  //SUM OF THE GIFT MUST > CONSTRAINT

  LOG(INFO) << "Number of constraints = " << solver.NumConstraints();

  // Maximize x + 10 * y.
  MPObjective* const objective = solver.MutableObjective();
  for(int i=0;i<day;i++)
  {
    for(int j=0;j<time_interval.size();j++)
    {
      objective->SetCoefficient(num_of_times[i][j],time_cost[j]);
      objective->SetCoefficient(buy_or_not[i][j],stone_for_gift[j]);
    }
  }
  objective->SetMinimization();

  const MPSolver::ResultStatus result_status = solver.Solve();
  // Check that the problem has an optimal solution.
  if (result_status != MPSolver::OPTIMAL) {
    LOG(FATAL) << "The problem does not have an optimal solution!";
  }

  LOG(INFO) << "Solution:";
  int final_sum=0;
  vector<vector<int>> sol_num_of_times;
  vector<vector<bool>> sol_activate;
  vector<vector<bool>> sol_buy_or_not;
  pair<vector<vector<int>>,pair<vector<vector<bool>>,vector<vector<bool>>>> result_for_return;
  for(int i=0;i<day;i++)
  {
    vector<int> snot;
    vector<bool> sa;
    vector<bool> sbon;
    LOG(INFO) << "";
    LOG(INFO) << "Day "<<i<<": ";
    LOG(INFO) << "";
    LOG(INFO) << "num_of_times[i][j]->solution_value();";
    for(int j=0;j<time_interval.size();j++)
    {
      LOG(INFO) << i<<" "<<j <<" "<<num_of_times[i][j]->solution_value();
      snot.push_back(num_of_times[i][j]->solution_value());
      final_sum += num_of_times[i][j]->solution_value()*10;
      final_sum += activate[i][j]->solution_value()*gift_coef[j].first;
      final_sum += buy_or_not[i][j]->solution_value()*(gift_coef[j].second-gift_coef[j].first);
    }
    LOG(INFO) << "";
    LOG(INFO) << "activate[i][j]->solution_value(); AND buy_or_not[i][j]->solution_value();";
    for(int j=0;j<time_interval.size();j++)
    {
      LOG(INFO) <<j <<": "<< activate[i][j]->solution_value()<<" "<< buy_or_not[i][j]->solution_value();
      sa.push_back(activate[i][j]->solution_value());
      sbon.push_back(buy_or_not[i][j]->solution_value());
    }
    sol_num_of_times.push_back(snot);
    sol_activate.push_back(sa);
    sol_buy_or_not.push_back(sbon);    
    LOG(INFO) << "";
  }
  LOG(INFO) << "Objective value(Cost) = " << objective->Value();
  LOG(INFO) << "Final value(Gain) = " << final_sum;
  LOG(INFO) << "\nAdvanced usage:";
  LOG(INFO) << "Problem solved in " << solver.wall_time() << " milliseconds";
  LOG(INFO) << "Problem solved in " << solver.iterations() << " iterations";
  LOG(INFO) << "Problem solved in " << solver.nodes()
            << " branch-and-bound nodes";
  cout<<endl;
  
  result_for_return = {sol_num_of_times,{sol_activate,sol_buy_or_not}};
  return result_for_return;
}
}  // namespace operations_research

int main(int argc, char** argv) {
  int event_day = 9;
  int basic_gift_per_time = 10;
  int gift_require = 6200;
  vector<int> interval_length = {5, 10, 20, 30, 40};
  vector<int> interval_cost = {0,10,15,20,20};
  vector<pair<int,int>> coef_gift = {{10,30},{20,60},{30,150},{30,150},{30,180}};
  vector<int> diamond = {20,40,100,100,120};
  
  pair<vector<vector<int>>,pair<vector<vector<bool>>,vector<vector<bool>>>> result_for_return = 
          operations_research::simple_mip_program(
          event_day, 
          basic_gift_per_time, 
          gift_require, 
          interval_length, 
          interval_cost, 
          coef_gift, 
          diamond);

  vector<vector<int>> sol_times = result_for_return.first;
  vector<vector<bool>> sol_activate = result_for_return.second.first;
  vector<vector<bool>> sol_buy = result_for_return.second.second;

  for(int i=0;i<event_day;i++)
  {
    int total_time = 0;
    for(int j=0;j<sol_times[i].size();j++)
    {
      total_time += sol_times[i][j];
    }
    cout<<"第"<<i+1<<"天: 通關"<<total_time<<"次";
    vector<int> have_buy;
    for(int j=0;j<sol_buy[i].size();j++)
    {
      if(sol_buy[i][j] == 1)
        have_buy.push_back(j);
    }

    if(have_buy.size() == 0)
      cout<<" 不買情報屋的饋贈\n";
    else if(have_buy.size() == 1)
    {
      cout<<" 買第 "<<have_buy[0]+1<<" 次\n";
    }
    else
    {
      cout<<" 買第";
      for(int j=0;j<have_buy.size();j++)
      {
        cout<<" "<<have_buy[j]+1;
      }
      cout<<" 次"<<endl;
    }
  }

  return EXIT_SUCCESS;
}