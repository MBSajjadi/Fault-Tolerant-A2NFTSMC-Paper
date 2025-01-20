function plotResults(t,x,Xd,u,uF,dHat,xBarResidue,...
    UpperBoundOfResidualsVector,LowerBoundOfResidualsVector,KHAT,tm)
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

%         if(i==1)
%             ylim([1 5])
%         elseif(i==2)
%             ylim([1 5])
%         else
%             ylim([-1 2])
%         end

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
        xlim([0 tm])

        if(i==5)
            ylim([-10 40]);
        elseif(i==4)
            ylim([-40 50])
        elseif (i==3)
            ylim([-15 40])
            yticks([-30 -15 0 15 30 50 65])
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
        xlim([0 tm])

        if(i==2)
            ylim([0 7])
            yticks([0 2 4 6 8])
        elseif(i==4)
            ylim([0 4])
        end
        
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
    
    %% Tracking Error
    
%     f6 = figure(6);
%     Name_e = {'e_{x}','e_{y}','e_{z}',...
%                       'e_{\phi}','e_{\theta}','e_{\psi}'};
%     for i=1:6
%         
%         subplot(3,2,i)
%         RandomColor = unifrnd(0,1,1,3);
%         plot(t,x(2*i-1,:)-Xd(2*i-1,:),'LineWidth',2,'Color',RandomColor);
%         xlabel('Time (s)','FontWeight','Bold')
%         legend(Name_e{i})
%         grid on
%         
%     end
%     
%     f4 = figure(4);
%     nameU = {'u_{1}','u_{2}','u_{3}','u_{4}','u_{5}','u_{6}'};
% 
%     for i=1:6
% 
%         subplot(3,2,i)
%         plot(t,u(i,:),'LineWidth',2)
%         grid on
%         xlabel('Time (s)','InterPrETer','Latex')
%         legend(nameU{i})
% 
%     end


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
        xlim([0 tm])
        
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

    %% Residuals Plots

    f1226 = figure(1226);
    f1226.Position = [600 480 800 700];
    n = 6;
    
    NameR = {'r_{x}','r_{y}','r_{z}','r_{\phi}','r_{\theta}','r_{\psi}'};
    
    MainUpper = UpperBoundOfResidualsVector;
    MainLower = LowerBoundOfResidualsVector;

for iter=1:10

    MainUpper(:,iter) = UpperBoundOfResidualsVector(:,11);
    MainLower(:,iter) = LowerBoundOfResidualsVector(:,11);

end

    for i=1:n
        
        subplot(3,2,i)
        plot(t,xBarResidue(i,:),'r','LineWidth',1.2)
        hold on
        grid on
        plot(t,MainUpper(i,:),'b','LineWidth',1.3)
        hold on
        plot(t,MainLower(i,:),'m','LineWidth',1.3)
        xlabel('Time (s)','InterPreter','Latex')
        ylabel(NameR{i},'FontWeight','Bold')
        xlim([0 tm])

        if(i==1)
             legend('r','r_{u}','r_{l}')
        else
            legend off
        end
        
        if(i==1)
            ylim([-0.2 0.4])
        elseif(i==4)
            ylim([-1 4])
        elseif(i==5)
            ylim([-3 2])
        end

    end

%     f110 = figure(110);
%     
%     L = 0.47/2;          % One-Half Length. Full Length equals 2*L = 47cm
%     b = 5.42e-5;    % Drag Force Coefficient
%     d = 1.1e-6;      % Drag Torque Coefficient
%     
%     FT = u(3,:)+u(4,:)+u(5,:)+u(6,:);
%     tauTheta = L*(u(5,:)-u(3,:));
%     tauPhi = L*(u(6,:)-u(4,:));
%     tauPsi = (d/(b))*(u(3,:)-u(4,:)+u(5,:)-u(6,:));
% 
%     MainS = [FT
%                    tauPhi
%                    tauTheta
%                    tauPsi];
% 
%     NmeU = {'F_{Thrust}','\tau_{\phi}','\tau_{\theta}','\tau_{\psi}'};
% 
%     for i=1:4
%         
%         subplot(2,2,i)
%         plot(t,MainS(i,:),'LineWidth',1.2)
%         grid on
%         xlabel('Time (s)','InterPreter','latex')
%         legend(NmeU{i})
% 
%     end

    %% Adaptive Gains

    f85 = figure(85);
    legendKHAT = {'$\hat{k_{\phi}}$','$\hat{k_{\theta}}$','$\hat{k_{\psi}}$'...
                             ,'$\hat{k_x}$','$\hat{k_y}$','$\hat{k_z}$'};

    for i=1:6

            subplot(3,2,i)
            plot(t,KHAT(i,:),'Color',[0.5 0.1 0.6],'LineWidth',1.4)
            grid on
            xlabel('Time (s)','InterPreter','Latex')
            ylabel(legendKHAT{i},'InterPreter','lAtex')

            if(i==1)
                legend('Estimated Switching Gain','InterPreter','Latex')
            else
                legend off
            end
            
            xlim([0 tm])

            if(i==1)
                ylim([-5 60])
            elseif(i==2)
                ylim([0 30])
            elseif(i==3)
                ylim([0 15])
            elseif(i==4)
                ylim([0 15])
            elseif(i==5)
                ylim([0 15])
            else
                ylim([0 10])
            end

    end

    %% Move Figures
    
    movegui(f1,'center')
    movegui(f5,'south')
    movegui(f8,'west')
    movegui(f3,'east')
    movegui(f1226,'north')
    movegui(f512,'southeast')
    movegui(f85,'southwest')
%     movegui(f110,'southwest')

end