%%%%%%%%%% Solve an LP to steer a Linear System %%%%%%%

function [x_seq,u_seq] = steerLinearSystembyLP(A,B,lower,upper,x_init,x_final,dt,numberOfSteps)

% Get dimensions

[stateDim,inputDim] = size(B);

solDim = stateDim*numberOfSteps + inputDim*(numberOfSteps-1);

% Initialize solution

x_seq = zeros(stateDim,numberOfSteps);
u_seq = zeros(inputDim,numberOfSteps-1);

% Indexing goes like this
% State vector, x = [x_1;x_2;...;x_stateDim]
% Input Vector u = [u_1;u_2;...;u_inputDim]
% sol_vector = [x_1(1),x_2(1),...,x_stateDim(1),
%   x_1(2),...,x_stateDim(numberOfSteps),u_1(1),u_2(1)
%   ,...,u_inputDim(1),....,u_inputDim(numberOfSteps-1)]

% Formula for indexing
% i^th state variable at time j, (j-1)*stateDim + i
% k^th input variable at time l, offset + (l-1)*inputDim + k
% offset = stateDim*numberOfSteps

offset = stateDim*numberOfSteps;

%% Set The Cost Function

% Simply minimize x_final

f = zeros(1,solDim);

for i=(numberOfSteps-1)*stateDim+1:(numberOfSteps)*stateDim
    
    f(i) = 1;
    
end

% -- it turned out that we dont really need that
% also weight the inputs, so that they don't blow up
% 
% for i=offset +1:offset + (numberOfSteps-1)*inputDim
%     
%     f(i) = 1e6;
%     
% end
% 
% 

%% Inequality Constraints

% Just bound the x_final

Aineq = zeros(stateDim,solDim);
bineq = -x_final;

k=1;

    
for state = 1:stateDim
   
        Aineq(k,(numberOfSteps-1)*stateDim+state) = -1;
        
        k = k+1;
    
end
    


%% Equality Constraints (this is the tricky part!!)

Aeq = zeros(numberOfSteps*stateDim,solDim);
beq = zeros(numberOfSteps*stateDim,1);

% Initial conditions

k=1;
for i=1:stateDim
    
    Aeq(k,i) = 1;
    beq(i) = x_init(i);
    k = k+1;
end


% Now constrain the system to obey the dynamics


constrainIndex= stateDim+1;

for timeIndex=2:numberOfSteps
           
    for stateIndex=1:stateDim
        
        Aeq(constrainIndex,(timeIndex-1)*stateDim + stateIndex) = 1;
        
        for Index=1:stateDim
            
            if(Index==stateIndex)
               
                 Aeq(constrainIndex,(timeIndex-2)*stateDim + Index) = -A(stateIndex,Index)*dt-1;
            else
                
                Aeq(constrainIndex,(timeIndex-2)*stateDim + Index) = -A(stateIndex,Index)*dt;
                
            end
        
            for inputIndex=1:inputDim
                            
                 Aeq(constrainIndex,offset+(timeIndex-2)*inputDim + inputIndex) = -B(stateIndex,inputIndex)*dt;
                
            end
                                  
        end
        constrainIndex = constrainIndex+1;
            
    end
    
end

%% Set the lower and upper bounds (copy-paste over variables)

lb = zeros(solDim,1);
ub = zeros(solDim,1);

% for states

for i=1:numberOfSteps
    
   
    for j=1:stateDim
        
        lb((i-1)*stateDim + j) = lower(j);
        ub((i-1)*stateDim + j) = upper(j);
        
    end
    
end

% for inputs


for i=1:numberOfSteps-1
    
   
    for j=1:inputDim
        
        lb(offset + (i-1)*inputDim + j) = lower(stateDim+j);
        ub(offset + (i-1)*inputDim + j) = upper(stateDim+j);
       
    end
    
end

%% Solve The LP and arrange the solution

sol = linprog(f,Aineq,bineq,Aeq,beq,lb,ub);


for i=1:numberOfSteps

    for j=1:stateDim
    
   x_seq(j,i) = sol((i-1)*stateDim + j);
    end

end


for i=1:numberOfSteps-1

    for j=1:inputDim
    
   u_seq(j,i) = sol(offset+ (i-1)*inputDim + j);
    end

end


end