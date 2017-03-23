%%  Import and ready the data.
format short
clc; 
clear;

data = importdata('orig/sensor_output_6.csv');
data1 = data;

data1( 1,: ) = [];
tmp = data1;
tmp( :, ~any(tmp,1) ) = [];  %columns
tmp1 = tmp;
tmp1( ~any(tmp1,2), : ) = [];  %rows
tmp2 = tmp1;
tmp2( ~all(tmp2,2), : ) = [];  %rows
data2 = tmp2;

for i = 0:10
    data2(1,:)=[];
end

%%  Scale the dung beetle unit from 1 dung beetle lenght, to [cm]
%   The dung beetle i 18.6 mm long. Which is the same as 1.86cm
%   So we ned to multiply all the data with 1.86 to find the actual
%   understable data.
data2 = data2.*1.86;

%%  Make the rotation matrix for projection.
%   The goal is to project the data down to the x-axis, so that the start
%   and the end is on the x-axis. This will make it better for presentation
%   and to compare the different data sets.
clc;
%   Find the body part that everything will be oriented at.
%   Part 24 = Abdomen, Part 25 = Thorax, Part 26 = Head
Part = 24;
start_point = [ data2( 1, Part*3+3) data2( 1, Part*3+2) ];
end_point = [ data2( size(data2,1), Part*3+3) data2( size(data2,1), Part*3+2) ];
%new_end_point = [ end_point-start_point 0 ]
new_end_point = [end_point-start_point];



%   Here we find the angle that we need to rotate 
%   the points, to put it on the x-axis.
r = -acos( dot(new_end_point, [1 0])/( norm(new_end_point)*norm( [1 0] ) ) );

%rotation_matrix = [ cos(r) -sin(r) 0; sin(r) cos(r) 0; 0 0 1 ];
rotation_matrix = [ 1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r) ];
%rotation_matrix  = [ cos(r) 0 sin(r); 0 1 0; -sin(r) 0 cos(r) ];

%%  Rotate the data, and translate back to zero.
for i = 0:26
    data3(:,i*3+1:i*3+3) = [ data2(:,i*3+1) data2(:,i*3+2)-start_point(2) data2(:,i*3+3)-start_point(1) ]*rotation_matrix;
end

figure(1)
hold on
grid on
%axis([-1 100 -22.5 22.5 -0.1 1.1])
for i = 24:24
    plot3( data2( :, i*3+3), data2( :, i*3+2), -data2( :, i*3+1),'linewidth',3 )
end
%close(figure(1))

figure(2)
hold on
grid on

xlabel('[cm]')
ylabel('[cm]')
zlabel('[cm]')

set(gca,'fontsize',20)
%axis([-1 75 -22.5 22.5 -0.1 1.1])

for i = 24:24
    plot3( data3( :, i*3+3), data3( :, i*3+2), -data3( :, i*3+1),'linewidth',3 )
end
%close(figure(2))

%%  Run the statistic analysis on the data.
%%  Analysis the data of the height
%   We are interested in knowning what the mean, variance and the standard
%   deviation of the data is.

%   Making a vector that holds the height value the body part parts.
A = [ -data3( :, 24*3+1) ];
%   It is now possible to find the mean, and variance of the height.
meanA = mean(A);
varA = var(A);
stdA = std(A);
%anovaA = anova1(A)

%%  Analysis the data of the walking on the x-axis.
%   We are interested in knowning what the mean, variance and the standard
%   deviation of the data is.

%   Making a vector that holds the data value for the body part on
%   the x-axis
B = [ data3( :, 24*3+2) ];
%   It is now possible to find the mean, and variance of the height.
meanB = mean(B);
varB = var(B);
stdB = std(B);
%anovaB = anova1(B);

%%  This one is useless for the data.
%   Making a vector that holds the data value for the body part on
%   the y-axis
%C = [ data3( :, 24*3+3)];
%   It is now possible to find the mean, and variance of the height.
%meanC = mean(C)
%varC = var(C)
%stdC = std(C)
%anovaC = anova1(C)

%%  Max deviation of data.
%   Find the abs of the min and max, and report that for A and B.
%   Height first:
    maxDA = max([abs(max(A)) abs(min(A))]);
%   Max deviation from the a-axis
    maxDB = max([abs(max(B)) abs(min(B))]);

%%  Collect all the output into two vectors used for the latex table.

outputHeight = [ meanA varA stdA maxDA ]

outputWalk = [ meanB varB stdB maxDB norm(new_end_point) ]

%%  Plot of the transient period. Needs to have the whole data set though
figure(3)
hold on
plot( -data3( 1:size(data3,1), 24*3+1),'linewidth',3, 'color', 'g' )
plot( -data3( 1:50, 24*3+1),'linewidth',3, 'color', 'r' )
xlabel('')
ylabel('[cm]')
set(gca,'fontsize',20)
axis([-1 size(data3,1) -0.1 1])
grid on
%close(figure(3))


