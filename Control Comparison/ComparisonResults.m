clc;clear;close all;

set(groot, 'DefaultAxesFontName', 'Times New Roman'); % Font name for x and y ticks
set(groot, 'DefaultAxesFontSize', 6); % Font size for x and y ticks
set(groot, 'DefaultAxesFontWeight', 'bold'); % Font weight for x and y ticks
set(groot, 'DefaultLegendFontName', 'Times New Roman'); % Font name for legend
set(groot, 'DefaultLegendFontSize', 6); % Font size for legend
set(groot, 'DefaultLegendFontWeight', 'bold'); % Font weight for legend

Ts = 0.01;
t = 0:Ts:40;

load('XD');

xDOBSMC = load('xDOBSMC');
xDOBSMC = xDOBSMC.x;

xDOBTSMC = load('xDOBTSMC');
xDOBTSMC = xDOBTSMC.x;

xANFTSMC = load('xANFTSMC');
xANFTSMC = xANFTSMC.x;

xDOBSMC = [(180/pi)*xDOBSMC(5,:)
                    xDOBSMC(7,:)
                    xDOBSMC(9,:)
                    xDOBSMC(11,:)];         % PSI X Y Z

xDOBTSMC = [(180/pi)*xDOBTSMC(5,:)
                    xDOBTSMC(7,:)
                    xDOBTSMC(9,:)
                    xDOBTSMC(11,:)];         % PSI X Y Z

xANFTSMC = [(180/pi)*xANFTSMC(5,:)
                    xANFTSMC(7,:)
                    xANFTSMC(9,:)
                    xANFTSMC(11,:)];         % PSI X Y Z

XD = [(180/pi)*XD(5,:)
          XD(7,:)
          XD(9,:)
          XD(11,:)];

f1 = figure(1);
% f1.Position = [100 80 700 600];
Name1 = {'DOBSMC','DOBTSMC','Proposed','\psi_{d}'};
xdName = {'\psi_{d}','x_{d}','y_{d}','z_{d}'};
yLabelName = {'\psi (Degree)','x(m)','y(m)','z(m)'};

for i=1:4
    
    subplot(2,2,i)
    plot(t,xDOBSMC(i,:),'b','LineWidth',1.1)
    hold on
    grid on
    plot(t,xDOBTSMC(i,:),'m','LineWidth',1.1)
    hold on
    plot(t,xANFTSMC(i,:),'r','LineWidth',1.1)
    hold on
    plot(t,XD(i,:),'k--','LineWidth',1)
    legend(Name1{1},Name1{2},Name1{3},xdName{i})
    ylabel(yLabelName{i},'FontWeight','BOld')
    xlim([0 40])

    if(i==2 || i==3)
        ylim([-15 15])
    elseif(i==1)
        ylim([0 20])
    end
end




set(groot, 'DefaultAxesFontName', 'Times New Roman'); % Font name for x and y ticks
set(groot, 'DefaultAxesFontSize', 8); % Font size for x and y ticks
set(groot, 'DefaultAxesFontWeight', 'bold'); % Font weight for x and y ticks
set(groot, 'DefaultLegendFontName', 'Times New Roman'); % Font name for legend
set(groot, 'DefaultLegendFontSize', 8); % Font size for legend
set(groot, 'DefaultLegendFontWeight', 'bold'); % Font weight for legend

f2 = figure(2);
plot3(xDOBSMC(2,:),xDOBSMC(3,:),xDOBSMC(4,:),'b','LineWidth',1.1)
hold on
grid on
plot3(xDOBTSMC(2,:),xDOBTSMC(3,:),xDOBTSMC(4,:),'m','LineWidth',1.1)
hold on
plot3(xANFTSMC(2,:),xANFTSMC(3,:),xANFTSMC(4,:),'r','LineWidth',1.1)
hold on
plot3(XD(2,:),XD(3,:),XD(4,:),'k--','lINeWidth',1)
xlabel('x(m)','InterPreter','LatEx')
ylabel('y(m)','InterPreter','LatEx')
zlabel('z(m)','InterPreter','LatEx')
LegendOutput = legend('DOBSMC','DOBTSMC','Proposed','Target Trajectory');
LegendOutput.Position = [0.5 0.3 0.1 0.1];