#include "ortools/linear_solver/linear_solver.h"
using namespace std;
namespace operations_research {

void simple_mip_program() {
  // Create the mip solver with the CBC backend.
  MPSolver solver("simple_mip_program",
                  MPSolver::CBC_MIXED_INTEGER_PROGRAMMING);

  const double infinity = solver.infinity();
  // x and y are integer non-negative variables.
  vector<vector<MPVariable*>> num_of_times;
  vector<vector<MPVariable*>> activate;
  vector<vector<MPVariable*>> buy_or_not;
  for(int i=0;i<9;i++)
  {
    vector<MPVariable*> temp0;
    temp0.push_back (solver.MakeIntVar(0,5,to_string(i)+"0"));
    temp0.push_back (solver.MakeIntVar(0,5,to_string(i)+"1"));
    temp0.push_back (solver.MakeIntVar(0,10,to_string(i)+"2"));
    temp0.push_back (solver.MakeIntVar(0,10,to_string(i)+"3"));
    temp0.push_back (solver.MakeIntVar(0,10,to_string(i)+"4"));
    num_of_times.push_back(temp0);
    vector<MPVariable*> temp1;
    vector<MPVariable*> temp2;
    for(int j=0;j<5;j++)
    {
      
      temp1.push_back(solver.MakeBoolVar("a("+to_string(i)+","+to_string(j)+")"));
      temp2.push_back(solver.MakeBoolVar("b("+to_string(i)+","+to_string(j)+")"));
    }
    activate.push_back(temp1);
    buy_or_not.push_back(temp2);
  }

  LOG(INFO) << "Number of variables = " << solver.NumVariables();
  vector<int> coef = {5,10,20,30,40};
  vector<int> coef2 = {5,10,10,10};
  vector<MPConstraint*> ActivateConstraint;
  for(int i=0;i<9;i++)
  {
    for(int j=0;j<4;j++)
    {
      MPConstraint* temp = solver.MakeRowConstraint(0,infinity,"act day"+to_string(i)+": "+to_string(j)+","+to_string(i+1));
      temp->SetCoefficient(activate[i][j],1);
      temp->SetCoefficient(activate[i][j+1],-1);
      ActivateConstraint.push_back(temp);
    }
    for(int j=0;j<4;j++)
    {
      MPConstraint* temp = solver.MakeRowConstraint(0,infinity,"");
      temp->SetCoefficient(activate[i][j],coef2[j]);
      temp->SetCoefficient(num_of_times[i][j+1],-1);
      ActivateConstraint.push_back(temp);
    }
    for(int j=0;j<5;j++)
    {
      MPConstraint* temp = solver.MakeRowConstraint(0,infinity,"act day"+to_string(i)+": "+to_string(j));
      for(int k=0;k<=j;k++){
        temp->SetCoefficient(num_of_times[i][k],1);
      }
      temp->SetCoefficient(activate[i][j],-coef[j]);
      ActivateConstraint.push_back(temp);
    }

    //Must activate before buy
    for(int j=0;j<5;j++)
    {
      MPConstraint* temp = solver.MakeRowConstraint(0,infinity,""); 
      temp->SetCoefficient(activate[i][j],1);
      temp->SetCoefficient(buy_or_not[i][j],-1);
      ActivateConstraint.push_back(temp);
    }
    MPConstraint* temp = solver.MakeRowConstraint(0,infinity,"");
    temp->SetCoefficient(activate[i][0],10);
    temp->SetCoefficient(activate[i][1],10);
    temp->SetCoefficient(activate[i][2],10);
    temp->SetCoefficient(activate[i][3],10);
    temp->SetCoefficient(num_of_times[i][0],-1);
    temp->SetCoefficient(num_of_times[i][1],-1);
    temp->SetCoefficient(num_of_times[i][2],-1);
    temp->SetCoefficient(num_of_times[i][3],-1);
    temp->SetCoefficient(num_of_times[i][4],-1);
    ActivateConstraint.push_back(temp);
  }

  vector<vector<int>> coef_gift = {{10,20},{20,40},{30,120},{30,150},{30,180}};
  MPConstraint* gift_require = solver.MakeRowConstraint(6200,infinity,"Requirement");
  for(int i=0;i<9;i++)
  {
    for(int j=0;j<5;j++)
    {
      gift_require->SetCoefficient(num_of_times[i][j],10);
      gift_require->SetCoefficient(activate[i][j],coef_gift[j][0]);
      gift_require->SetCoefficient(buy_or_not[i][j],coef_gift[j][1]);
    }
  }
  //SUM OF THE GIFT MUST > CONSTRAINT

  LOG(INFO) << "Number of constraints = " << solver.NumConstraints();

  vector<int> diamond = {20,40,100,100,120};
  vector<int> coef_gift_std = {0,10,15,20,20};
  // Maximize x + 10 * y.
  MPObjective* const objective = solver.MutableObjective();
  for(int i=0;i<9;i++)
  {
    for(int j=0;j<5;j++)
    {
      objective->SetCoefficient(num_of_times[i][j],coef_gift_std[j]);
      objective->SetCoefficient(buy_or_not[i][j],diamond[j]);
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
  for(int i=0;i<9;i++)
  {
    LOG(INFO) << "";
    LOG(INFO) << "Day "<<i<<": ";
    LOG(INFO) << "";
    LOG(INFO) << "num_of_times[i][j]->solution_value();";
    for(int j=0;j<5;j++)
    {
      LOG(INFO) << i<<" "<<j <<" "<<num_of_times[i][j]->solution_value();
      final_sum += num_of_times[i][j]->solution_value()*10;
      final_sum += activate[i][j]->solution_value()*coef_gift[j][0];
      final_sum += buy_or_not[i][j]->solution_value()*coef_gift[j][1];
    }
    LOG(INFO) << "";
    LOG(INFO) << "activate[i][j]->solution_value(); AND buy_or_not[i][j]->solution_value();";
    for(int j=0;j<5;j++)
    {
      LOG(INFO) <<j <<": "<< activate[i][j]->solution_value()<<" "<< buy_or_not[i][j]->solution_value();
    }    
    LOG(INFO) << "";
  }
  LOG(INFO) << "Objective value(Cost) = " << objective->Value();
  LOG(INFO) << "Final value(Gain) = " << final_sum;
  LOG(INFO) << "\nAdvanced usage:";
  LOG(INFO) << "Problem solved in " << solver.wall_time() << " milliseconds";
  LOG(INFO) << "Problem solved in " << solver.iterations() << " iterations";
  LOG(INFO) << "Problem solved in " << solver.nodes()
            << " branch-and-bound nodes";
}
}  // namespace operations_research

int main(int argc, char** argv) {
  operations_research::simple_mip_program();
  return EXIT_SUCCESS;
}
