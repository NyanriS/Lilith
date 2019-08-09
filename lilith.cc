#include<vector>
#include "ortools/linear_solver/linear_solver.h"
using namespace std;

vector<int> gift = {
    60,80, //1
    130,150,170,190, //5
    260,280,300,320,380,400,420,440, //13
    390,410,430,450,510,530,550,570,510,530,550,570,630,650,670,690, //29
    520,540,560,580,640,660,680,700,640,660,680,700,760,780,800,820,
    670,690,710,730,790,810,830,850,790,810,830,850,910,930,950,970 //61
};
vector<int> diamond = {
    0,20,
    50,70,90,110,
    200,220,240,260,300,320,340,360,
    400,420,440,460,500,520,540,560,500,520,540,560,600,620,640,660,
    600,620,640,660,700,720,740,760,700,720,740,760,800,820,840,860,
    720,740,760,780,820,840,860,880,820,840,860,880,920,940,960,980
};
namespace operations_research {
void simple_mip_program() {

  // Create the mip solver with the CBC backend.
  MPSolver solver("simple_mip_program",
                  MPSolver::CBC_MIXED_INTEGER_PROGRAMMING);

  const double infinity = solver.infinity();

    vector<MPVariable*> variable;
    for(int i=0;i<62;i++)
    {
        variable.push_back(solver.MakeIntVar(0,7,to_string(i)));
    }
    LOG(INFO) << "Number of variables = " << solver.NumVariables();

    MPConstraint* const c0 = solver.MakeRowConstraint(0, 9, "c0");
    MPConstraint* const c1 = solver.MakeRowConstraint(3800, infinity, "c1");
    for(int i=0;i<62;i++)
    {
        c0->SetCoefficient(variable[i],1);
        c1->SetCoefficient(variable[i],gift[i]);
    }
    MPObjective* const objective = solver.MutableObjective();
    for(int i=0;i<62;i++)
    {
        objective->SetCoefficient(variable[i],diamond[i]);
    }
    objective->SetMinimization();

    const MPSolver::ResultStatus result_status = solver.Solve();
    // Check that the problem has an optimal solution.
    if (result_status != MPSolver::OPTIMAL) {
        LOG(FATAL) << "The problem does not have an optimal solution!";
    }

    LOG(INFO) << "Solution:";
    LOG(INFO) << "Objective value = " << objective->Value();
    int sum=0;
    for(int i=0;i<62;i++)
    {
        if(variable[i]->solution_value() >0){
            LOG(INFO) <<"Variable "<<i<<" Gift = "<< gift[i] <<" * " << variable[i]->solution_value()<<" Diamond = "<<diamond[i];
            sum +=  variable[i]->solution_value() * gift[i];
        }
    }
    LOG(INFO) << "Gift = " << sum;


}
}

int main(int argc, char** argv) {
  operations_research::simple_mip_program();
  return EXIT_SUCCESS;
}