%%  
clf;
clear;

data = csvread('22_gait4.csv');

for i = 1:50
    data(1,:)=[];
end
a = size(data,1)-51
for i = size(data,1)-a:size(data,1)
    data(1,:)=[];
end

legFL=data(:,1);
legML=data(:,2);
legHL=data(:,3);
legFR=data(:,4);
legMR=data(:,5);
legHR=data(:,6);

legFL = legFL + 5;
legML = legML + 4;
legHL = legHL + 3;
legFR = legFR + 2;
legMR = legMR + 1;
legHR = legHR + 0;

legFL(legFL==5) = NaN;
legML(legML==4) = NaN;
legHL(legHL==3) = NaN;
legFR(legFR==2) = NaN;
legMR(legMR==1) = NaN;
legHR(legHR==0) = NaN;

hold all
stairs(legFL,'LineWidth',60,'Color',[1.0 0 0])
stairs(legML,'LineWidth',60,'Color',[0.75 0 0])
stairs(legHL,'LineWidth',60,'Color',[0.5 0 0])
stairs(legFR,'LineWidth',60,'Color',[0 0 1.0])
stairs(legMR,'LineWidth',60,'Color',[0 0 0.75])
stairs(legHR,'LineWidth',60,'Color',[0 0 0.5])
axis([0,size(data,1),0.5,6.5])

xlabel('')
ylabel('Contact with ground')
set(gca,'fontsize',20)
