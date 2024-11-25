% plot raw csv data
% for post flight processing 
%
filename = 'x-extracted.csv';
t = readtable(filename);
x_axis_vals = t{:,1};
y_axis_vals = t{:,2}; 

plot(y_axis_vals);
