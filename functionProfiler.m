function functionProfiler(fun, iter)
% FUNCTIONPROFILER: helper function to run a function multiple times and
% evaluate what the average running time is for the function
% 
%   FUNCTIONPROFILER(FUN,ITER) runs
%   a function iter times and outputs the average running time
% 
%   INPUTS
%       fun   function handle to test runtime
%       iter  number of times to run function
%
%   Cornell University
%   MAE 4180: Autonomous Mobile Robots
%   Final Competition
%   Pu, Kenneth (kp295)

% Run function iter times
avg_time = 0;
max_time = 0;
min_time = 99999;
for i=1:iter
    tic;feval(fun);t=toc;
    if t>max_time
        max_time = t;
    end
    if t<min_time
        min_time = t;
    end
    avg_time = avg_time+t;
end

% Output Results
fprintf('# Iterations: %d\n  >> Max: %f seconds\n  >> Min: %f seconds\n  >> Avg: %f seconds\n', iter,max_time,min_time,avg_time/iter);