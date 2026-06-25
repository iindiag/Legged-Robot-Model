% First created MATLAB script for the project
% Demonstrates the basics, for learning use

clc, clearvars % Clears command window, clears workspace

x = 1:10; % Defines x as a horizontal array vector 1-10

x'; % Transposed x so that it is now a vertical array

y = [3 6 40 10]; % Acts as a list, space used instead of comma (but comma
                % can still be used)

A = [1 3; 2 -10]; % 2x2 matrix, rows separated by ;

A.^2; % The . means that operation is performed element-wise

A(1,2); % Accessing 1st row 2nd column

% 0 is not used as an index, end can be used to access the last value
% : means all when accessing a row/column

% Example problem 1: anonymous functions
clc, clearvars

x = linspace(0,5);
y = (-(x-3).^2) + 10;

%plot(x,y,'*')

%[MaxVal, I] = max(y) % Numerically errors due to linspace, 
                     % analytical solution is 10
                     % Finds max values and corresponding index

%x_MaxVal = x(I) % Finds the x value that corresponds to the max y val from
                % the index

% Defining a custom function:

y = @(x) ((-(x-3).^2)) + 10; % y is now an anonymous function

y(20.7);

% Example problem 2: plotting
clc, clearvars

x = linspace(-10,10);
y1 = (-(x-3).^2) + 10;
y2 = (-(x-3).^2) + 15;
y3 = (-(x-5).^2) + 10;

% To plot all 3 on the same figure, use the command hold on between
% plotting the lines

%figure(1)
%subplot(2,2,1)
%plot(x,y1,'*','Color','b','MarkerSize',10)
%xlabel('x')
%ylabel('y')
%title('Title')
%grid on

% Example problem 3: logic
% y = sin(x), what percent of y-values are greater than 0.8 for x = 0 to
% 10?
clc, clearvars

x = linspace(0,10,1000000);
y = sin(x);

%plot(x,y,'.'), hold on, plot([0 10],[0.8 0.8],'-r')

y_greater = y > 0.8;
percent = sum(y_greater) / length(y) * 100 % Sum is used for y_greater as
                                           % it returns 1s or 0s
%%
% Example problem 4
% Section 1: if statement
clc, clearvars

A = randi(5,1,10); % Uniform, discrete dist. 10 random number 1-5

if sum(A == 3) >= 3
    disp('wow!')
end

%% 
% 2 % signs introduce a new section, can be run and act independently
% Section 2: for loop
clc, clearvars

A = randi(5,1,10);

num3 = 0
tic % Starts timer
for i = 1:length(A)
    if A(i) == 3
        num3 = num3 +1
    end
end
toc % Ends timer
% for loops are generally slower than if statement methods

if num3 >= 3
    disp('wow!')
end

%%
% Section 3: while loop, runs whilst the condition is true

%%
% Section 4: custom functions
clc, clearvars

% Can also create this function in a separate script, the names of the
% files and function must match when called
function [z_final] = reduce_z(z_initial)

z = z_initial;

while z > z_initial / 2
    z = z - 1;
end

z_final = z

end

reduce_z(100)