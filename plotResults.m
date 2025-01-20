function plotResults(t,x,Xd,u,uF,dHat,tmax)
    %% A. Plot the States
    
    set(groot, 'DefaultAxesFontName', 'Times New Roman'); % Font name for x and y ticks
    set(groot, 'DefaultAxesFontSize', 12); % Font size for x and y ticks
    set(groot, 'DefaultAxesFontWeight', 'bold'); % Font weight for x and y ticks
    set(groot, 'DefaultLegendFontName', 'Times New Roman'); % Font name for legend
    set(groot, 'DefaultLegendFontSize', 12); % Font size for legend
    set(groot, 'DefaultLegendFontWeight', 'bold'); % Font weight for legend

    f1 = figure(1);
    Name = {'x','y','z'};
    ylabelNamex = {'x(m)','y(m)','z(m)'};
    NameReference = {'x_{d}','y_{d}','z_{d}'};
                 
    for i=1:3
        
        subplot(3,1,i)
        plot(t,(x(2*i-1,:)),'r','LineWidth',1.4)
        hold on
        plot(t,(Xd(2*i-1,:)),'k--','LineWidth',1)
        grid on
        xlabel('Time (s)','InterPreter','Latex')
        legend(Name{i},NameReference{i},'FontWeight','bold')
        ylabel(ylabelNamex{i},'InterPreter','latEx')

        if(i==1)
            ylim([2 5])
        elseif(i==2)
            ylim([2 5])
        else
            ylim([-1 3])
        end

    end
    
    f8 = figure(8);
    Name = {'\phi','\theta','\psi'};
    NameReference = {'\phi_{d}','\theta_{d}','\psi_{d}'};
    ii = 0;

    for i=3:5
        
        ii = ii + 1;
        subplot(3,1,ii);

        plot(t,180/pi*(x(2*i+1,:)),'r','LineWidth',1.4)
        hold on
        plot(t,180/pi*((Xd(2*i+1,:))),'k--','LineWidth',1)
        grid on
        xlabel('Time (s)','InterPreter','Latex')
        legend(Name{ii},NameReference{ii})
        ylabel([Name{ii}, num2str(' (Degree)')])
        xlim([0 tmax])

        if(i==5)
            ylim([-10 40]);
        elseif(i==4)
            ylim([-40 40])
        elseif (i==3)
            ylim([-20 60])
%             yticks([-30 -15 0 15 30 50 65])
        end

    end

    %% Control Signals Plot
    
    f3 = figure(3);
    U_Name = {'F_{1}','F_{2}','F_{3}','F_{4}'};
    ForceVector = abs([u(3,:)
                          u(4,:)
                          u(5,:)
                          u(6,:)]);

    yLabelName = {'$F_1 (N)$','$F_2 (N)$','$F_3 (N)$','$F_4 (N)$'};

    for i=1:4
    
        subplot(2,2,i)
        plot(t,ForceVector(i,:),'r','LineWidth',1.2)
        xlabel('Time(s)','InterPreter','Latex')
        grid on
        legend(U_Name{i},'FontWeight','bold')
        ylabel(yLabelName{i},'InterPreter','LatEx')
        xlim([0 tmax])

%         if(i==2)
%             ylim([0 7])
%             yticks([0 2 4 6 8])
%         elseif(i==4)
%             ylim([0 4])
%         end
        
    end
   
    %% Desired 3D Trajectory
    
    f5 = figure(5);
    X = x(1,:);
    y = x(3,:);
    z = x(5,:);
    
    plot3(X,y,z,'r','LineWidth',1.3)
    xlabel('x (m)','InterPreter','Latex')
    ylabel('y (m)','InterPreter','Latex')
    zlabel('z (m)','InterPreter','Latex')
    hold on
    plot3(Xd(1,:),Xd(3,:),Xd(5,:),'b--','LineWidth',1.1)
    grid on
    LegendOutput = legend('UAV Trajectory','Target Trajectory');
    LegendOutput.Position = [0.5 0.3 0.1 0.1];

    %% External Faults Plot
    
    f512 = figure(512);

    ufLineWidth = 1.3;
    dHatLineWidth = 1;
    ylabelNames = {'u_{f\phi} (rad.s^-2)','u_{f\theta} (rad.s^-2)','u_{f\psi} (rad.s^-2)',...
                             'u_{fx} (m.s^-2)','u_{fy} (m.s^-2)','u_{fz} (m.s^-2)'};

    for i=1:6
        
        subplot(3,2,i)
        plot(t,dHat(i,:),'b','LineWidth',dHatLineWidth)
        hold on
        grid on
        plot(t,uF(i,:),'r--','LineWidth',ufLineWidth)
        xlabel('Time (s)','InterPreter','LatEx')
        ylabel(ylabelNames{i})
        xlim([0 tmax])
        
        if(i==1)
            legend('$\hat{u_f}$','$u_{f}$','InterPreter','Latex')
        end

%         if(i==1)
%             ylim([-20 150])
%         elseif(i==2)
%             ylim([-10 5])
%         elseif(i==3)
%             ylim([-20 60])
%         elseif(i==4)
%             ylim([-2 5])
%         elseif(i==5)
%             ylim([-2 5])
%         elseif(i==6)
%             ylim([-4 2])
%         end

    end

    %% Move Figures
    
    movegui(f1,'center')
    movegui(f5,'south')
    movegui(f8,'west')
    movegui(f3,'east')
    movegui(f512,'southeast')

end