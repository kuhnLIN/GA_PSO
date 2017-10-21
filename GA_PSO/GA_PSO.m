%% GA 优化 PSO
%% 清空环境
clc;
clear
close all
%% 参数初始化
lenchrom=7;    %字符串长度（个体长度），染色体编码长度
pc=0.7;        %设置交叉概率，本例中交叉概率是定值，若想设置变化的交叉概率可用表达式表示，或从写一个交叉概率函数，例如用神经网络训练得到的值作为交叉概率
pm=0.3;        %设置变异概率，同理也可设置为变化的

%粒子群算法中的两个参数
c1 = 1.49445;
c2 = 1.49445;

maxgen=20;   % 进化次数  
popsize=30; %种群规模

%粒子更新速度
Vmax=1;
Vmin=-1;

%种群
popmax=50;
popmin=-50;

% 变量取值范围
bound=[popmin popmax;popmin popmax;popmin popmax;popmin popmax;popmin popmax;popmin popmax;popmin popmax];  %变量范围

% 优化粒子数目
par_num=7;

%% 产生初始粒子和速度
for i=1:popsize
    %随机产生一个种群
    pop(i,:)=popmax*rands(1,par_num);    %初始种群
    V(i,:)=rands(1,par_num);  %初始化速度
    %计算适应度
    fitness(i)=fun(pop(i,:));   %染色体的适应度
end

%找最好的染色体
[bestfitness bestindex]=min(fitness);
zbest=pop(bestindex,:);   %全局最佳
gbest=pop;    %个体最佳
fitnessgbest=fitness;   %个体最佳适应度值
fitnesszbest=bestfitness;   %全局最佳适应度值

%% 迭代寻优
for i=1:maxgen
    i
    for j=1:popsize
        
        %速度更新 PSO选择更新
        V(j,:) = V(j,:) + c1*rand*(gbest(j,:) - pop(j,:)) + c2*rand*(zbest - pop(j,:));
        V(j,find(V(j,:)>Vmax))=Vmax;
        V(j,find(V(j,:)<Vmin))=Vmin;
        
        %种群更新 PSO选择更新
        pop(j,:)=pop(j,:)+0.5*V(j,:);
        pop(j,find(pop(j,:)>popmax))=popmax;
        pop(j,find(pop(j,:)<popmin))=popmin;
        
        % 交叉操作 GA
        GApop=Cross(pc,lenchrom,pop,popsize,bound);
        
        % 变异操作 GA变异
        GApop=Mutation(pm,lenchrom,GApop,popsize,[i maxgen],bound);
        
        pop=GApop; % GA pop --> PSO pop
      
        % 适应度值 --> 约束条件
         if 0.072*pop(j,1)+0.063*pop(j,2)+0.057*pop(j,3)+0.05*pop(j,4)+0.032*pop(j,5)+0.0442*pop(j,6)+0.0675*pop(j,7)<=264.4
            if 128*pop(j,1)+78.1*pop(j,2)+64.1*pop(j,3)+43*pop(j,4)+58.1*pop(j,5)+36.9*pop(j,6)+50.5*pop(j,7)<=69719
                    fitness(j)=fun(pop(j,:));
               end
         end

        %个体最优更新
        if fitness(j) < fitnessgbest(j)
            gbest(j,:) = pop(j,:);
            fitnessgbest(j) = fitness(j);
        end
        
        %群体最优更新
        if fitness(j) < fitnesszbest
            zbest = pop(j,:);
            fitnesszbest = fitness(j);
        end
        
    end
    
    yy(i)=fitnesszbest;     
end

%% 结果
disp '*************best particle number****************'
zbest

%%
plot(yy,'linewidth',2);
grid on
title(['适应度曲线  ' '终止代数＝' num2str(maxgen)]);
xlabel('进化代数');ylabel('适应度');
