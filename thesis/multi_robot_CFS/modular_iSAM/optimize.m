function [] = optimize(mode, graph, robot_id, var_2)

global multi_isam;
global isam initial;
global init_x init_y init_theta;

isam = [isam isam isam isam];

if isequal(mode, 'batch')
    batchOptimizer = LevenbergMarquardtOptimizer(graph, initial);
    initial = batchOptimizer.optimize();    
end

isam(robot_id).update(graph, initial);
result = isam(robot_id).calculateEstimate();
init_x = result.at(var_2).x;
init_y = result.at(var_2).y;
init_theta = result.at(var_2).theta;

graph = NonlinearFactorGraph;
initial = Values;

