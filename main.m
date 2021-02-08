clc; clearvars; close all;
%% Define variables
x_vec = [ falcon.State('x',   0,  11000, 1e-4);...
          falcon.State('v',   0,  12.5, 1) ];

u_vec = falcon.Control('i', 0,  30, 1);

tf = falcon.Parameter('FinalTime', 1400, 0, 1400, 1e-3);
phi = falcon.Parameter('phi', 9.7692, 5.0800, 12.7000, 1);

modeloutput = [...
    falcon.Output('Ft');...
    falcon.Output('Fa');...
    falcon.Output('Fr');...
    falcon.Output('Fg');...
    falcon.Output('theta')];

%% Create the model
mdl = falcon.SimulationModelBuilder('DT1', x_vec, u_vec, phi);
mdl.addSubsystem(@mdl_dt1_pista, {'x', 'v', 'i', 'phi'}, {'mdl_out'});               
mdl.SplitVariable('mdl_out', {'x_dot','v_dot', 'Ft', 'Fa', 'Fr', 'Fg', 'theta'}.')
mdl.setStateDerivativeNames('x_dot','v_dot');
mdl.setOutputs(modeloutput);
mdl.Build();

%% Create the Lagrange cost
lgc = falcon.PathConstraintBuilder('LCost', [], x_vec(2), u_vec(1), phi, @lagrange_cost);
lgc.Build;

%% Create the path function
pti = falcon.PathConstraintBuilder('LimCor', [], x_vec(2), u_vec(1), phi, @i_max);
pti.Build;

%% Create the problem
problem = falcon.Problem('DT1_B1');

tau = linspace(0,1,1001);

phase = problem.addNewPhase(@DT1, x_vec, tau, 0, tf);
phase.Model.setModelParameters(phi);

phase.addNewControlGrid(u_vec, tau); 

phase.Model.setModelOutputs(modeloutput);

phase.setInitialBoundaries([0;0]);
phase.setFinalBoundaries([10080;0],[10080;12.5]);

pathc = phase.addNewPathConstraint(@LimCor,falcon.Constraint('Iup', -inf, 0, 1),tau);
pathc.setParameters(phi);

lagc = phase.addNewLagrangeCost(@LCost,falcon.Cost('Cost',1e-4));

lagc.setParameters(phi);

%% Solve
problem.Solve();

%% Display results

voltage = 0.24*problem.ControlValues + ...
    0.119*(problem.StateValues(2,:)*phi.Value/0.254);

for j=1:length(voltage)
   if problem.ControlValues(j) <= 0.01
      voltage(j) = 0;
   end
end

energy = trapz(problem.RealTime/3600,voltage.*problem.ControlValues)/1000;
efficiency = (problem.StateValues(1,end)/energy(end))/1000;

fprintf('Efficiency [km/kWh]..: %.2f \n', efficiency)
fprintf('Gear ratio []........: %.2f \n', phi.Value)
fprintf('Final time [s] ......: %.1f \n', tf.Value)

