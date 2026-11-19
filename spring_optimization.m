function spring_optimization()
    % Define initial guess for the design variables: x1 = d, x2 = D, x3 = N
    % Ensure they are within the specified bounds
    x0 = [0.05, 0.5, 10]; 

    % Define lower and upper bounds for x = [x1, x2, x3]
    lb = [0.05, 0.25, 2];
    ub = [2.0, 1.3, 15];

    % Options for fmincon
    options = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ...
        'Display', 'iter', ...
        'ConstraintTolerance', 1e-6, ...
        'OptimalityTolerance', 1e-6);

    % Solve the constrained optimization problem using fmincon
    [x_opt, f_opt, exitflag, output] = fmincon(@objectiveFunction, x0, [], [], [], [], lb, ub, @nonlcon, options);

    % Display the results
    disp('--------------------------------------------------');
    disp('Optimization Results:');
    fprintf('Optimal wire diameter (x1 = d):        %.6f\n', x_opt(1));
    fprintf('Optimal mean coil diameter (x2 = D):   %.6f\n', x_opt(2));
    fprintf('Optimal number of active coils (x3 = N): %.6f\n', x_opt(3));
    fprintf('Minimum weight/objective value f(x):   %.6f\n', f_opt);
    disp('--------------------------------------------------');
end

% Objective function: Minimize f(x) = x1^2 * x2 * (x3 + 2)
function f = objectiveFunction(x)
    x1 = x(1);
    x2 = x(2);
    x3 = x(3);
    
    f = (x1^2) * x2 * (x3 + 2);
end

% Nonlinear constraints function: g(x) <= 0
function [c, ceq] = nonlcon(x)
    x1 = x(1);
    x2 = x(2);
    x3 = x(3);

    % Inequality constraints (g(x) <= 0) based on the assignment image
    g1 = (x(3) * x(2)^3) / (71785 * x1^4) - 1;
    
    term1 = (4 * x2^2 - x1 * x2) / (12566 * (x2 * x1^3 - x1^4));
    term2 = 1 / (5108 * x1^2);
    g2 = 1 - (term1 + term2);
    
    g3 = (140.45 * x1) / (x3 * x2^2) - 1;
    
    g4 = (x1 + x2) / 1.5 - 1;

    c = [g1; g2; g3; g4];
    
    % No equality constraints
    ceq = [];
end