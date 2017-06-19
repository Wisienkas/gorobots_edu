%%  Plot all the sensors in 2D and 3D
clf;
clc; 
clear;

data1 = csvread('output.csv');
data1( 1,: ) = [];
tmp = data1;
tmp( :, ~any(tmp,1) ) = [];  %columns
tmp1 = tmp;
tmp1( ~any(tmp1,2), : ) = [];  %rows
tmp2 = tmp1;
tmp2( ~all(tmp2,2), : ) = [];  %rows
data2 = tmp2;

for i = 0:40
    data2(1,:)=[];
end

figure(1)
clf;
hold all
title('Test - Biological - Swing between AEP & PEP')
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');

axis([-0.6 0.4 -0.8 0 0.5 1.5])
%axis([-3.5 3.5 -3.5 3.5 -3.5 3.5])
%axis([-1 10 -1 10 -5 5])
grid on

for i = 0:11
    plot3( data2( :, i*3+3),data2( :, i*3+2), -data2( :, i*3+1) )
end

%%

for i = 24:26
    plot3( data2( 1, i*3+3),data2( 1, i*3+2), -data2( 1, i*3+1), '*g' )
    plot3( data2( size(data2,1), i*3+3),data2( size(data2,1), i*3+2), -data2( size(data2,1), i*3+1), '*r' )
end
    plot3( data2( :, 24*3+3),data2( :, 24*3+2), -data2( :, 24*3+1),'-r' )
    plot3( data2( :, 25*3+3),data2( :, 25*3+2), -data2( :, 25*3+1),'-g' )
    plot3( data2( :, 26*3+3),data2( :, 26*3+2), -data2( :, 26*3+1),'-b' ) 
